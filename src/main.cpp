#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_controller.hpp"
#include "robot_interface.hpp"
#include "user_controller.hpp"

namespace fs = std::filesystem;

static booster_deploy::RobotController *g_robot_controller = nullptr;

static void handle_signal(int)
{
    if (g_robot_controller)
    {
        g_robot_controller->StopLogging();
    }
    std::exit(0);
}

int main(int argc, char const *argv[])
{
    // ---- DDS domain / interface ----
    // no extra args: local loopback (booster_mujoco sim, domain 1, interface lo)
    // args: <config> <network_interface> (real robot convention: domain 0)
    if (argc < 2)
    {
        booster::robot::ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    }

    // ---- controller ----
    fs::path param_folder = fs::current_path() / "../params";
    std::string config_file = (param_folder / "k1_config.yaml").string();

    YAML::Node cfg = YAML::LoadFile(config_file);
    auto *ctrl = new booster_deploy::K1OnnxController(config_file, booster_deploy::k1_joint_names());

    // policy-rate data log (DataTamer MCAP); empty string disables it
    std::string data_log_path = cfg["data_log_path"] ? cfg["data_log_path"].as<std::string>() : "";
    auto *robot_controller = new booster_deploy::RobotController(ctrl, data_log_path);

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    g_robot_controller = robot_controller;
    
    std::unique_ptr<booster::robot::b1::B1LocoClient> client;  // 提升到 main 作用域
    if (argc >= 2)
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
