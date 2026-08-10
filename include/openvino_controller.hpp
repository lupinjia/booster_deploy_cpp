#pragma once

// OpenVINO inference backend (K1OpenVINOController).
//
// Reads deployment parameters from the .onnx metadata (same file as the
// onnxruntime backend, via a throwaway onnxruntime session) and runs
// inference with OpenVINO on the very same .onnx file:
//   core.read_model(onnx) -> compile_model(model, "CPU") -> infer_request.
// This keeps the shared ObservationBuilder pipeline and all config identical
// to the ONNX / LibTorch paths.

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <openvino/openvino.hpp>
#include <yaml-cpp/yaml.h>

#include "observation_builder.hpp"
#include "onnx_metadata.hpp"
#include "user_controller.hpp"

namespace booster_deploy
{

class K1OpenVINOController : public BasicUserController
{
public:
    K1OpenVINOController(const std::string &cfg_file, std::vector<std::string> robot_joint_names)
        : cfg_file_(cfg_file), robot_joint_names_(std::move(robot_joint_names))
    {
    }

    void loadParam() override
    {
        auto cfg = YAML::LoadFile(cfg_file_);
        try
        {
            policy_name_ = cfg["policy_name"].as<std::string>();
            dt = cfg["dt"].as<float>();
            stand_kp_ = cfg["stand_kp"].as<float>();
            stand_kd_ = cfg["stand_kd"].as<float>();
            ctrl_kp_fallback_ = cfg["ctrl_kp"].as<float>();
            ctrl_kd_fallback_ = cfg["ctrl_kd"].as<float>();
            remote_timeout_ms_ = cfg["remote_timeout_ms"].as<float>(100.f);
        }
        catch (const std::exception &e)
        {
            std::cerr << "[k1_openvino] failed to parse " << cfg_file_ << ": " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    void loadPolicy() override
    {
        std::filesystem::path policy_file = policy_name_;
        if (policy_file.is_relative())
        {
            policy_file = std::filesystem::current_path() / "../models" / policy_name_;
        }
        if (!std::filesystem::exists(policy_file))
        {
            std::cerr << "[k1_openvino] policy file not found: " << policy_file << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // ---- parse deployment params from the .onnx metadata ----
        {
            Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "booster_deploy_cpp");
            Ort::SessionOptions session_options;
            Ort::Session onnx_session(env, policy_file.c_str(), session_options);
            metadata_ = std::make_unique<OnnxMetadata>(load_onnx_metadata(onnx_session));
        }
        if (metadata_->num_joints() != static_cast<int>(robot_joint_names_.size()))
        {
            std::cerr << "[k1_openvino] metadata joints " << metadata_->num_joints()
                      << " != robot joints " << robot_joint_names_.size() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        metadata_->print_summary();

        // ---- OpenVINO: read the same .onnx, compile on CPU ----
        policy_ = core_.read_model(policy_file.string());
        compiled_model_ = core_.compile_model(policy_, "CPU");
        infer_request_ = compiled_model_.create_infer_request();

        robot_ = std::make_unique<BoosterRobotInterface>(robot_joint_names_);
        obs_builder_ = std::make_unique<ObservationBuilder>(*metadata_, *robot_);

        stand_pos = metadata_->default_joint_pos;  // policy order
        start_pos = stand_pos;
        stand_pos_real_.assign(robot_joint_names_.size(), 0.f);
        for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
        {
            stand_pos_real_[obs_builder_->meta2real()[i]] = stand_pos[i];
        }
        jpos_des = stand_pos_real_;
        action_scale_ = metadata_->action_scale;

        kp_ = std::vector<float>(robot_joint_names_.size(), ctrl_kp_fallback_);
        kd_ = std::vector<float>(robot_joint_names_.size(), ctrl_kd_fallback_);
        if (metadata_->joint_stiffness.size() == metadata_->joint_names.size() &&
            metadata_->joint_damping.size() == metadata_->joint_names.size())
        {
            for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
            {
                int real_idx = obs_builder_->meta2real()[i];
                kp_[real_idx] = metadata_->joint_stiffness[i];
                kd_[real_idx] = metadata_->joint_damping[i];
            }
        }
        std::cout << "[k1_openvino] policy loaded: " << policy_file
                  << "  (obs_dim=" << metadata_->actor_obs_dim
                  << ", joints=" << metadata_->num_joints() << ")" << std::endl;
    }

    void reset(BoosterRobotInterface &robot) override
    {
        (void)robot;
        obs_builder_->reset();
        obs_builder_->last_action_.assign(obs_builder_->num_policy_joints(), 0.f);
        actions_.assign(obs_builder_->num_policy_joints(), 0.f);
    }

    void GetInput(BoosterRobotInterface &robot, Gamepad &gamepad) override
    {
        (void)robot;
        obs_builder_->command_[0] = -gamepad.ly;
        obs_builder_->command_[1] = -gamepad.lx;
        obs_builder_->command_[2] = -gamepad.rx;
    }

    void Calculate() override
    {
        if (!robot_ || !obs_builder_)
        {
            std::cerr << "[k1_openvino] Calculate called before loadPolicy" << std::endl;
            std::abort();
        }

        std::vector<float> obs = obs_builder_->compute();
        if (static_cast<int>(obs.size()) != metadata_->actor_obs_dim)
        {
            std::cerr << "[k1_openvino] obs dim " << obs.size()
                      << " != actor_obs_dim " << metadata_->actor_obs_dim << std::endl;
            std::abort();
        }

        // single-input flattened observation (same layout as the ONNX path)
        ov::Tensor obs_tensor(ov::element::f32, {1, static_cast<size_t>(obs.size())});
        float *obs_data = obs_tensor.data<float>();
        for (size_t i = 0; i < obs.size(); ++i)
        {
            obs_data[i] = obs[i];
        }
        infer_request_.set_input_tensor(0, obs_tensor);
        infer_request_.infer();
        ov::Tensor output_tensor = infer_request_.get_output_tensor();
        float *output_data = output_tensor.data<float>();

        int n = metadata_->num_joints();
        actions_.assign(n, 0.f);
        for (int i = 0; i < n; ++i)
        {
            actions_[i] = output_data[i];
        }
        obs_builder_->last_action_ = actions_;

        jpos_des = stand_pos_real_;
        const auto &meta2real = obs_builder_->meta2real();
        for (int i = 0; i < n; ++i)
        {
            jpos_des[meta2real[i]] += actions_[i] * action_scale_[i];
        }
    }

    // apply a zero-observation warmup inference
    void DummyCalculate() override
    {
        if (!metadata_) return;
        std::vector<float> obs(metadata_->actor_obs_dim, 0.f);
        ov::Tensor obs_tensor(ov::element::f32, {1, static_cast<size_t>(obs.size())});
        float *obs_data = obs_tensor.data<float>();
        for (size_t i = 0; i < obs.size(); ++i)
        {
            obs_data[i] = obs[i];
        }
        infer_request_.set_input_tensor(0, obs_tensor);
        infer_request_.infer();
    }

    std::vector<float> GetLog() override
    {
        std::vector<float> log;
        log.push_back(obs_builder_->command_[0]);
        log.push_back(obs_builder_->command_[1]);
        log.push_back(obs_builder_->command_[2]);
        return log;
    }

    const std::vector<float> &kp() const override { return kp_; }
    const std::vector<float> &kd() const override { return kd_; }
    float stand_kp() const override { return stand_kp_; }
    float stand_kd() const override { return stand_kd_; }
    const std::vector<float> &stand_pos_real() const override { return stand_pos_real_; }

    BoosterRobotInterface &robot() override { return *robot_; }
    float remote_timeout_ms() const override { return remote_timeout_ms_; }

private:
    std::string cfg_file_;
    std::vector<std::string> robot_joint_names_;

    std::string policy_name_;
    float stand_kp_ = 60.f, stand_kd_ = 5.f;
    float ctrl_kp_fallback_ = 20.f, ctrl_kd_fallback_ = 0.5f;
    float remote_timeout_ms_ = 100.f;

    ov::Core core_;
    std::shared_ptr<ov::Model> policy_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    std::unique_ptr<OnnxMetadata> metadata_;
    std::unique_ptr<BoosterRobotInterface> robot_;
    std::unique_ptr<ObservationBuilder> obs_builder_;

    std::vector<float> action_scale_;
    std::vector<float> stand_pos_real_;
    std::vector<float> kp_, kd_;
    std::vector<float> actions_;
};

} // namespace booster_deploy
