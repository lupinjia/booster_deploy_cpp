#pragma once

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "gamepad.hpp"
#include "observation_builder.hpp"
#include "onnx_metadata.hpp"
#include "robot_interface.hpp"

namespace booster_deploy
{

class BasicUserController
{
public:
    BasicUserController() = default;
    virtual ~BasicUserController() = default;

    virtual void loadParam() = 0;
    virtual void loadPolicy() = 0;
    virtual void reset(BoosterRobotInterface &robot) = 0;
    virtual void GetInput(BoosterRobotInterface &robot, Gamepad &gamepad) = 0;
    virtual void Calculate() = 0;
    virtual std::vector<float> GetLog() = 0;

    // shared robot interface / deployment params (implemented by each backend)
    virtual BoosterRobotInterface &robot() = 0;
    virtual void DummyCalculate() = 0;
    virtual float stand_kp() const = 0;
    virtual float stand_kd() const = 0;
    virtual const std::vector<float> &kp() const = 0;
    virtual const std::vector<float> &kd() const = 0;
    virtual const std::vector<float> &stand_pos_real() const = 0;
    virtual float remote_timeout_ms() const = 0;

    void save_jpos(BoosterRobotInterface &robot)
    {
        std::copy(robot.jpos.begin(), robot.jpos.end(), start_pos.begin());
    }

    float dt = 0.02f;
    std::vector<float> start_pos;  // pose captured on entering stand
    std::vector<float> stand_pos;  // stand pose target
    std::vector<float> jpos_des;   // current joint position targets
};

// ---------------------------------------------------------------------------
// K1 velocity locomotion controller driven by an mjlab-exported ONNX policy.
//
// All observation terms, scales, history, action scale, joint order, default
// pose and PD gains are read automatically from the ONNX metadata (mirroring
// booster_deploy's OnnxPolicy).  The yaml only supplies the policy file, the
// control period, transition poses and DDS parameters.
// ---------------------------------------------------------------------------

class K1OnnxController : public BasicUserController
{
public:
    K1OnnxController(const std::string &cfg_file, std::vector<std::string> robot_joint_names)
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
            std::cerr << "[k1_onnx] failed to parse " << cfg_file_ << ": " << e.what() << std::endl;
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
            std::cerr << "[k1_onnx] policy not found: " << policy_file << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // ---- onnxruntime session ----
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "booster_deploy_cpp");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_ = std::make_unique<Ort::Session>(*env_, policy_file.c_str(), session_options);

        // ---- metadata -> deployment config ----
        metadata_ = std::make_unique<OnnxMetadata>(load_onnx_metadata(*session_));
        if (metadata_->num_joints() != static_cast<int>(robot_joint_names_.size()))
        {
            std::cerr << "[k1_onnx] metadata joints " << metadata_->num_joints()
                      << " != robot joints " << robot_joint_names_.size() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        metadata_->print_summary();

        // policy input/output names
        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = session_->GetInputNameAllocated(0, allocator);
        auto output_name = session_->GetOutputNameAllocated(0, allocator);
        input_name_ = input_name.get();
        output_name_ = output_name.get();

        // ---- robot interface + obs builder ----
        robot_ = std::make_unique<BoosterRobotInterface>(robot_joint_names_);
        obs_builder_ = std::make_unique<ObservationBuilder>(*metadata_, *robot_);

        // ---- metadata-driven joint config ----
        stand_pos = metadata_->default_joint_pos;   // policy order
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
            // per-joint PD gains, mapped from policy order to real robot order
            for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
            {
                int real_idx = obs_builder_->meta2real()[i];
                kp_[real_idx] = metadata_->joint_stiffness[i];
                kd_[real_idx] = metadata_->joint_damping[i];
            }
        }
        std::cout << "[k1_onnx] policy loaded: " << policy_file
                  << "  (input: " << input_name_ << ", obs_dim=" << metadata_->actor_obs_dim
                  << ", output: " << output_name_ << ")" << std::endl;
    }

    void reset(BoosterRobotInterface &robot) override
    {
        obs_builder_->reset();
        obs_builder_->last_action_.assign(obs_builder_->num_policy_joints(), 0.f);
        actions_.assign(obs_builder_->num_policy_joints(), 0.f);
    }

    void GetInput(BoosterRobotInterface &robot, Gamepad &gamepad) override
    {
        (void)robot;  // state is read from the shared robot interface
        // velocity command from gamepad (same convention as go2_deploy /
        // booster remote_control_service)
        obs_builder_->command_[0] = -gamepad.ly;
        obs_builder_->command_[1] = -gamepad.lx;
        obs_builder_->command_[2] = -gamepad.rx;
    }

    void Calculate() override
    {
        if (!robot_ || !obs_builder_)
        {
            std::cerr << "[k1_onnx] Calculate called before loadPolicy" << std::endl;
            std::abort();
        }

        std::vector<float> obs = obs_builder_->compute();
        if (static_cast<int>(obs.size()) != metadata_->actor_obs_dim)
        {
            std::cerr << "[k1_onnx] obs dim " << obs.size()
                      << " != actor_obs_dim " << metadata_->actor_obs_dim << std::endl;
            std::abort();
        }

        // ---- inference ----
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> input_shape{1, metadata_->actor_obs_dim};
        std::vector<int64_t> output_shape{1, metadata_->num_joints()};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, obs.data(), obs.size(), input_shape.data(), input_shape.size());

        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1,
                                     output_names, 1);
        float *action_raw = outputs[0].GetTensorMutableData<float>();

        // raw (pre-scale) action for the next obs step
        int n = metadata_->num_joints();
        actions_.assign(action_raw, action_raw + n);
        obs_builder_->last_action_ = actions_;

        // dof targets in real robot order: default pose + scaled action
        jpos_des = stand_pos_real_;
        const auto &meta2real = obs_builder_->meta2real();
        for (int i = 0; i < n; ++i)
        {
            jpos_des[meta2real[i]] += actions_[i] * action_scale_[i];
        }
        (void)output_shape;
    }

    // apply a zero-observation warmup inference
    void DummyCalculate() override
    {
        if (!session_ || !metadata_) return;
        std::vector<float> obs(metadata_->actor_obs_dim, 0.f);
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> shape{1, metadata_->actor_obs_dim};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, obs.data(), obs.size(), shape.data(), shape.size());
        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
    }

    std::vector<float> GetLog() override
    {
        std::vector<float> log;
        log.push_back(obs_builder_->command_[0]);
        log.push_back(obs_builder_->command_[1]);
        log.push_back(obs_builder_->command_[2]);
        return log;
    }

    // per-joint PD gains for the CTRL state (real robot order)
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
    float remote_timeout_ms_ = 100.f;  // watchdog: stop if remote silent this long

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_, output_name_;

    std::unique_ptr<OnnxMetadata> metadata_;
    std::unique_ptr<BoosterRobotInterface> robot_;
    std::unique_ptr<ObservationBuilder> obs_builder_;

    std::vector<float> action_scale_;
    std::vector<float> stand_pos_real_;
    std::vector<float> kp_, kd_;
    std::vector<float> actions_;
};

} // namespace booster_deploy
