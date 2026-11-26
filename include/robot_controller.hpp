#pragma once

// RobotController framework (adapted from go2_deploy robot_controller.hpp).
// Owns the DDS channels (rt/low_state, rt/remote_controller_state,
// rt/joint_ctrl), the finite state machine and the control threads.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/RemoteControllerState.h>

#include "conversion.hpp"
#include "data_logger.hpp"
#include "gamepad.hpp"
#include "local_joystick.hpp"
#include "periodic_task.hpp"
#include "robot_interface.hpp"
#include "state_machine.hpp"
#include "user_controller.hpp"

namespace booster_deploy
{

class RobotController
{
public:
    RobotController(BasicUserController *ctrl, const std::string &data_log_path = "")
        : ctrl_(ctrl), data_log_path_(data_log_path)
    {
    }

    void loadParam() { ctrl_->loadParam(); }
    void loadPolicy() { ctrl_->loadPolicy(); }

    void InitDdsModel()
    {
        lowcmd_publisher = std::make_shared<booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>>(
            booster::robot::b1::kTopicJointCtrl);
        lowstate_subscriber = std::make_shared<booster::robot::ChannelSubscriber<booster_interface::msg::LowState>>(
            booster::robot::b1::kTopicLowState,
            std::bind(&RobotController::LowStateMessageHandler, this, std::placeholders::_1));
        if (!use_local_joystick_)
        {
            remote_subscriber = std::make_shared<booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState>>(
                remote_topic_,
                std::bind(&RobotController::RemoteControllerMessageHandler, this, std::placeholders::_1));
        }

        lowcmd_publisher->InitChannel();
        lowstate_subscriber->InitChannel();
        if (remote_subscriber)
        {
            remote_subscriber->InitChannel();
        }

        init_low_cmd();
    }

    void set_remote_topic(const std::string &topic) { remote_topic_ = topic; }

    // Real-robot mode: read the local joystick device directly instead of
    // subscribing to the DDS remote_controller_state topic.  Chosen by main()
    // based on argc (argc >= 2 => real robot).
    void set_use_local_joystick(bool use) { use_local_joystick_ = use; }
    void set_joystick_device(const std::string &dev) { joystick_device_ = dev; }

    // Flush and close the MCAP data log (safe to call from a signal handler).
    void StopLogging()
    {
        if (data_logger_)
        {
            data_logger_->stop();
        }
    }

    void StartControl()
    {
        const auto duration = std::chrono::milliseconds(100);

        // real-robot: poll the local joystick device (before waiting for START,
        // since the START button itself comes from this device)
        if (use_local_joystick_)
        {
            local_joystick_ = std::make_unique<LocalJoystick>(joystick_device_);
            if (!local_joystick_->open())
            {
                std::cerr << "[robot_controller] cannot open joystick device "
                          << joystick_device_ << std::endl;
                std::exit(EXIT_FAILURE);
            }
            std::cout << "Local joystick: " << local_joystick_->deviceName() << std::endl;
            joystick_task_ = legged::common::createPeriodicTask(
                "LocalJoystick", joystick_poll_us_,
                std::bind(&RobotController::LocalJoystickStep, this), -1, SCHED_OTHER, 20);
        }

        // wait for the gamepad START button to begin
        std::cout << "Press START button to start!" << std::endl;
        auto last_report = std::chrono::steady_clock::now();
        while (true)
        {
            std::this_thread::sleep_for(duration);
            TickGamepad();
            if (gamepad_.start.on_press) break;
            if (std::chrono::steady_clock::now() - last_report > std::chrono::seconds(1))
            {
                std::cout << "  [wait] low_state_frames=" << low_state_frame_count_
                          << " remote_frames=" << remote_frame_count_
                          << " start=" << gamepad_.start.pressed << std::endl;
                last_report = std::chrono::steady_clock::now();
            }
        }
        std::cout << "Start!" << std::endl;

        Damping();
        ctrl_dt_micro_sec_ = static_cast<uint64_t>(ctrl_->dt * 1e6);

        // policy-rate data log (MCAP); disabled when data_log_path_ is empty
        if (!data_log_path_.empty())
        {
            std::string mcap_path = data_log_path_ + ".mcap";
            data_logger_ = std::make_unique<DataLogger>(
                mcap_path, static_cast<int>(ctrl_->robot().jpos.size()), ctrl_->obs_dim());
            std::cout << "Data log enabled: " << mcap_path << std::endl;
        }

        // 50Hz control task
        control_task_ = legged::common::createPeriodicTask(
            "ControlStep", static_cast<float>(ctrl_dt_micro_sec_),
            std::bind(&RobotController::ControlStep, this), -1, SCHED_OTHER, 20);

        // 500Hz low-level command task
        std::this_thread::sleep_for(duration);
        StartSendCmd();
        std::cout << "Start Send Cmd!" << std::endl;

        while (true)
        {
            std::this_thread::sleep_for(duration);
        }
    }

protected:
    std::shared_ptr<booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>> lowcmd_publisher;
    std::shared_ptr<booster::robot::ChannelSubscriber<booster_interface::msg::LowState>> lowstate_subscriber;
    std::shared_ptr<booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState>> remote_subscriber;

    booster_interface::msg::LowCmd cmd_;
    Gamepad gamepad_;
    SimpleStateMachine state_machine_;
    BasicUserController *ctrl_;

    std::mutex state_mutex_, cmd_mutex_;
    legged::common::PeriodicTaskPtr control_task_, cmd_task_, joystick_task_;
    uint64_t ctrl_dt_micro_sec_ = 20000;
    std::vector<float> compute_time_ms_;
    std::string remote_topic_ = "rt/remote_controller_state";
    std::string data_log_path_;
    std::unique_ptr<DataLogger> data_logger_;
    uint64_t step_count_ = 0;
    bool use_local_joystick_ = false;
    std::string joystick_device_ = "/dev/input/js0";
    float joystick_poll_us_ = 10000.0f;  // 100Hz device polling
    std::unique_ptr<LocalJoystick> local_joystick_;

private:
    // 500Hz: consume low-level robot state
    void LowStateMessageHandler(const void *message)
    {
        const auto *state = static_cast<const booster_interface::msg::LowState *>(message);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ctrl_->robot().SetState(*state);
            ++low_state_frame_count_;
        }
    }

    std::atomic<uint64_t> low_state_frame_count_{0};

    void RemoteControllerMessageHandler(const void *message)
    {
        const auto *remote = static_cast<const booster_interface::msg::RemoteControllerState *>(message);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            gamepad_.set_raw(*remote);
            ++remote_frame_count_;
        }
    }

    // Real-robot: poll the local joystick device and feed the gamepad with the
    // same RemoteControllerState semantics.  If the device is gone (unplugged),
    // keep the last known state (no new commands arrive).
    void LocalJoystickStep()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!local_joystick_ || !local_joystick_->poll())
        {
            return;
        }
        booster_interface::msg::RemoteControllerState remote;
        remote.lx(local_joystick_->lx());
        remote.ly(local_joystick_->ly());
        remote.rx(local_joystick_->rx());
        remote.ry(local_joystick_->ry());
        remote.a(local_joystick_->a());
        remote.b(local_joystick_->b());
        remote.x(local_joystick_->x());
        remote.y(local_joystick_->y());
        remote.lb(local_joystick_->lb());
        remote.rb(local_joystick_->rb());
        remote.back(local_joystick_->back());
        remote.start(local_joystick_->start());
        remote.lt(local_joystick_->lt());
        remote.rt(local_joystick_->rt());
        remote.hat_u(local_joystick_->hatUp());
        remote.hat_d(local_joystick_->hatDown());
        remote.hat_l(local_joystick_->hatLeft());
        remote.hat_r(local_joystick_->hatRight());
        gamepad_.set_raw(remote);
        ++remote_frame_count_;
    }

    std::atomic<uint64_t> remote_frame_count_{0};

    void init_low_cmd()
    {
        int n = static_cast<int>(ctrl_->robot().jpos.size());
        cmd_.cmd_type(booster_interface::msg::SERIAL);
        cmd_.motor_cmd().resize(n);
        for (int i = 0; i < n; ++i)
        {
            auto &m = cmd_.motor_cmd()[i];
            m.mode(0);
            m.q(0.f);
            m.dq(0.f);
            m.tau(0.f);
            m.kp(0.f);
            m.kd(0.f);
            m.weight(0.f);
        }
    }

    void set_cmd()
    {
        auto &robot = ctrl_->robot();
        int n = static_cast<int>(robot.jpos.size());
        for (int i = 0; i < n; ++i)
        {
            auto &m = cmd_.motor_cmd()[i];
            m.q(robot.jpos_des[i]);
            m.dq(robot.jvel_des[i]);
            m.kp(robot.kp[i]);
            m.kd(robot.kd[i]);
            m.tau(robot.tau_ff[i]);
        }
    }

    // Tick the gamepad under the state mutex.
    void TickGamepad()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        gamepad_.tick();
    }

    void StartSendCmd()
    {
        cmd_task_ = legged::common::createPeriodicTask(
            "SendCmd", 2000.0f,
            [this]() {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                lowcmd_publisher->Write(&cmd_);
            },
            -1, SCHED_OTHER, 20);
    }

    void UpdateStateMachine()
    {
        // L1 + B  -> Stand
        // L1 + A  -> Ctrl
        // L1 + Y  -> Stop
        // RT / RB -> speed gear up / down (handled in the policy GetInput)
        auto &g = gamepad_;
        if (g.b.on_press && g.lb.pressed)
        {
            if (state_machine_.Stand()) StandCallback();
        }
        if (g.a.on_press && g.lb.pressed)
        {
            if (state_machine_.Ctrl()) CtrlCallback();
        }
        if (g.y.pressed && g.lb.pressed)
        {
            state_machine_.Stop();
        }
    }

    // 50Hz control step (one iteration; paced by PeriodicTask "ControlStep")
    void ControlStep()
    {
        auto start = std::chrono::high_resolution_clock::now();

        TickGamepad();
        UpdateStateMachine();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ctrl_->GetInput(ctrl_->robot(), gamepad_);
        }

        if (state_machine_.state == STATES::STAND)
        {
            Standing();
        }
        else if (state_machine_.state == STATES::DAMPING)
        {
            Damping();
        }
        else if (state_machine_.state == STATES::CTRL)
        {
            UserControlStep(true);
            if (ctrl_->enable_fall_safe() && CheckTermination())
            {
                state_machine_.Stop();
                Damping();
            }
        }

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            set_cmd();
        }

        auto stop = std::chrono::high_resolution_clock::now();
        float ms = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1000.f;
        compute_time_ms_.push_back(ms);

        LogDataStep(ms);

        if (compute_time_ms_.size() == 50)
        {
            float sum = 0, mx = 0, mn = 1e9f;
            for (float t : compute_time_ms_)
            {
                sum += t;
                mx = std::max(mx, t);
                mn = std::min(mn, t);
            }
            std::cout << "Performance: mean: " << sum / 50 << " ms; max: " << mx
                      << " ms; min: " << mn << " ms.  State: "
                      << static_cast<int>(state_machine_.state)
                      << "  remote_frames: " << remote_frame_count_
                      << "  lb:" << gamepad_.lb.pressed
                      << " rt:" << gamepad_.rt.pressed
                      << " a:" << gamepad_.a.pressed
                      << " start:" << gamepad_.start.pressed
                      << " ly:" << gamepad_.ly
                      << std::endl;
            compute_time_ms_.clear();
        }
    }

    // Record one 50Hz sample to the DataTamer MCAP log (if enabled).
    void LogDataStep(float compute_time_ms)
    {
        if (!data_logger_) return;

        auto &robot = ctrl_->robot();
        int n = static_cast<int>(robot.jpos.size());
        auto to_double = [](const std::vector<float> &v) {
            return std::vector<double>(v.begin(), v.end());
        };

        DataLogSample sample;
        sample.t = step_count_ * ctrl_->dt;  // sim time: nominal policy period
        sample.inference_time = compute_time_ms / 1000.0;
        sample.state = static_cast<double>(state_machine_.state);
        sample.joint_pos = to_double(robot.jpos);
        sample.joint_vel = to_double(robot.jvel);
        sample.feedback_torque = to_double(robot.tau);
        // no odometry on K1: root_pos_w / root_lin_vel_b stay zero
        auto quat = euler_to_quat(robot.rpy[0], robot.rpy[1], robot.rpy[2]);
        for (int i = 0; i < 4; ++i) sample.root_quat_w[i] = quat[i];
        for (int i = 0; i < 3; ++i) sample.root_ang_vel_b[i] = robot.gyro[i];
        sample.dof_targets = to_double(robot.jpos_des);
        if (!ctrl_->last_obs().empty())
        {
            sample.obs = to_double(ctrl_->last_obs());
        }
        if (!ctrl_->last_action().empty())
        {
            sample.action = to_double(ctrl_->last_action());
        }
        (void)n;

        data_logger_->log(sample);
        ++step_count_;
    }

    void StandCallback()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ctrl_->save_jpos(ctrl_->robot());
        }
        auto &robot = ctrl_->robot();
        robot.jpos_des = ctrl_->start_pos;
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        std::fill(robot.kp.begin(), robot.kp.end(), ctrl_->stand_kp());
        std::fill(robot.kd.begin(), robot.kd.end(), ctrl_->stand_kd());
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
    }

    void CtrlCallback()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ctrl_->reset(ctrl_->robot());
        }
        auto &robot = ctrl_->robot();
        robot.jpos_des = ctrl_->stand_pos_real();
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        robot.kp = ctrl_->kp();  // metadata per-joint gains
        robot.kd = ctrl_->kd();
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
        std::cout << "Ctrl enabled." << std::endl;
    }

    void Damping(float kd = 3.0f)
    {
        auto &robot = ctrl_->robot();
        std::fill(robot.jpos_des.begin(), robot.jpos_des.end(), 0.f);
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        std::fill(robot.kp.begin(), robot.kp.end(), 0.f);
        std::fill(robot.kd.begin(), robot.kd.end(), kd);
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
    }

    void Standing()
    {
        ctrl_->DummyCalculate();  // warmup the neural network
        state_machine_.Standing(*ctrl_);
        ctrl_->robot().jpos_des = ctrl_->jpos_des;
    }

    void UserControlStep(bool send_cmd)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ctrl_->Calculate();
        }
        if (send_cmd)
        {
            ctrl_->robot().jpos_des = ctrl_->jpos_des;
        }
    }

    bool CheckTermination()
    {
        return ctrl_->robot().projected_gravity[2] > -0.1f;
    }
};

} // namespace booster_deploy
