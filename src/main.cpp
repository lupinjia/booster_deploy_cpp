#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_controller.hpp"
#include "robot_interface.hpp"
#include "user_controller.hpp"

namespace fs = std::filesystem;

// FastDDS profile used by the on-robot booster services.
static constexpr const char *kRealDdsProfile =
    "/opt/booster/config/dds_configs/single-board_configs/fastdds_profile.xml";

static booster_deploy::RobotController *g_robot_controller = nullptr;

static void handle_signal(int)
{
    if (g_robot_controller)
    {
        g_robot_controller->StopLogging();
    }
    std::exit(0);
}

static void print_usage(const char *argv0)
{
    std::cout << "Usage: " << argv0 << " [real|sim]\n"
              << "  real (default): domain 0 via InitWithConfigPath(0, " << kRealDdsProfile << ")\n"
              << "  sim           : domain 1, interface lo (booster_mujoco)\n";
}

int main(int argc, char const *argv[])
{
    // ---- DDS mode ----
    // real (default): domain 0, fastdds_profile.xml (booster_camera uses SHM+UDP)
    // sim           : domain 1, interface lo
    bool is_real = true;
    if (argc >= 2)
    {
        const std::string mode = argv[1];
        if (mode == "real" || mode == "--real")
        {
            is_real = true;
        }
        else if (mode == "sim" || mode == "--sim")
        {
            is_real = false;
        }
        else
        {
            print_usage(argv[0]);
            return 1;
        }
        if (argc > 2)
        {
            print_usage(argv[0]);
            return 1;
        }
    }

    if (is_real)
    {
        if (!fs::exists(kRealDdsProfile))
        {
            std::cerr << "real-robot DDS profile not found: " << kRealDdsProfile << "\n";
            return 1;
        }
        booster::robot::ChannelFactory::Instance()->InitWithConfigPath(0, kRealDdsProfile);
        std::cout << "[booster_deploy] mode=real (domain 0, profile " << kRealDdsProfile << ")\n";
    }
    else
    {
        booster::robot::ChannelFactory::Instance()->Init(1, "lo");
        std::cout << "[booster_deploy] mode=sim (domain 1, interface lo)\n";
    }

    // ---- controller ----
    fs::path param_folder = fs::current_path() / "../params";
    std::string config_file = (param_folder / "k1_config.yaml").string();

    YAML::Node cfg = YAML::LoadFile(config_file);
    auto *ctrl = new booster_deploy::K1OnnxController(config_file, booster_deploy::k1_joint_names());

    // policy-rate data log (DataTamer MCAP); empty string disables it
    std::string data_log_path = cfg["data_log_path"] ? cfg["data_log_path"].as<std::string>() : "";
    auto *robot_controller = new booster_deploy::RobotController(ctrl, data_log_path);

    // real: read the local joystick device directly instead of the DDS
    // remote_controller_state topic; sim keeps the DDS subscription.
    robot_controller->set_use_local_joystick(is_real);
    if (cfg["joystick_device"])
    {
        robot_controller->set_joystick_device(cfg["joystick_device"].as<std::string>());
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    g_robot_controller = robot_controller;
    
    std::unique_ptr<booster::robot::b1::B1LocoClient> client;  // 提升到 main 作用域
    if (is_real)
    {
        client = std::make_unique<booster::robot::b1::B1LocoClient>();
        client->Init();
        int32_t ret = client->ChangeMode(booster::robot::RobotMode::kCustom);
        if (ret != booster::robot::kRpcStatusCodeSuccess) { 
          std::cerr << "ChangeMode failed: " << ret << "\n"; 
          return 1; 
        }
    }

    robot_controller->loadParam();
    robot_controller->loadPolicy();
    robot_controller->InitDdsModel();
    
    // start control loop
    robot_controller->StartControl();

    return 0;
}
