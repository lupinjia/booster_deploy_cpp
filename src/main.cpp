#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_controller.hpp"
#include "robot_interface.hpp"
#include "torch_controller.hpp"
#include "user_controller.hpp"

namespace fs = std::filesystem;

int main(int argc, char const *argv[])
{
    // ---- log folder (named by start time) ----
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d-%H%M%S");
    fs::path log_folder = fs::current_path() / "logs" / ss.str();
    fs::create_directories(log_folder);
    fs::path log_file_name = log_folder / "log.csv";

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

    // ---- controller (backend selected at runtime from the config yaml) ----
    fs::path param_folder = fs::current_path() / "../params";
    std::string config_file = (param_folder / "k1_config.yaml").string();

    YAML::Node cfg = YAML::LoadFile(config_file);
    std::string backend = cfg["backend"] ? cfg["backend"].as<std::string>() : "onnx";

    booster_deploy::BasicUserController *ctrl = nullptr;
    if (backend == "torch")
    {
        ctrl = new booster_deploy::K1TorchController(config_file, booster_deploy::k1_joint_names());
    }
    else
    {
        ctrl = new booster_deploy::K1OnnxController(config_file, booster_deploy::k1_joint_names());
    }
    std::cout << "Inference backend: " << backend << std::endl;
    auto *robot_controller = new booster_deploy::RobotController(log_file_name.string(), ctrl);
    
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
