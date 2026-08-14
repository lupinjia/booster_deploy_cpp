#pragma once

// Policy-rate data logger backed by DataTamer (MCAP sink).
//
// Records one sample per control step (50Hz) with the same field layout as
// booster_deploy's data_logging.md: t, inference_time, joint_pos/joint_vel/
// feedback_torque (robot joint order), root_pos_w/root_quat_w/root_lin_vel_b/
// root_ang_vel_b, dof_targets, obs, action.
//
// DataTamer registers pointers to stable buffers owned by this class, so the
// logger snapshots are race-free even though the robot state is written by
// the 500Hz DDS callback thread: ControlStep fills the sample struct under
// the state mutex, log() copies it into the registered buffers, and
// takeSnapshot() captures them.

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <data_tamer/data_tamer.hpp>
#include <data_tamer/sinks/mcap_sink.hpp>

namespace booster_deploy
{

struct DataLogSample
{
    double t = 0.0;             // sim time (s), step_count * policy_dt
    double inference_time = 0.0;  // wall-clock inference time (s)
    double state = 0.0;         // state machine state (STATES enum value)

    std::vector<double> joint_pos;       // real-robot joint order
    std::vector<double> joint_vel;
    std::vector<double> feedback_torque;
    std::array<double, 3> root_pos_w{0, 0, 0};  // no odometry on K1: zero
    std::array<double, 4> root_quat_w{1, 0, 0, 0};  // [w, x, y, z]
    std::array<double, 3> root_lin_vel_b{0, 0, 0};  // no odometry on K1: zero
    std::array<double, 3> root_ang_vel_b{0, 0, 0};
    std::vector<double> dof_targets;     // position targets sent to PD
    std::vector<double> obs;             // actor_obs_dim (policy layout)
    std::vector<double> action;          // raw (pre-scale) policy output
};

class DataLogger
{
public:
    DataLogger(const std::string &mcap_path, int num_joints, int obs_dim)
    {
        sink_ = std::make_shared<DataTamer::MCAPSink>(mcap_path);
        DataTamer::ChannelsRegistry::Global().addDefaultSink(sink_);
        channel_ = DataTamer::ChannelsRegistry::Global().getChannel("policy_step");

        buf_.joint_pos.assign(num_joints, 0.0);
        buf_.joint_vel.assign(num_joints, 0.0);
        buf_.feedback_torque.assign(num_joints, 0.0);
        buf_.dof_targets.assign(num_joints, 0.0);
        buf_.action.assign(num_joints, 0.0);
        buf_.obs.assign(obs_dim, 0.0);

        channel_->registerValue("t", &buf_.t);
        channel_->registerValue("inference_time", &buf_.inference_time);
        channel_->registerValue("state", &buf_.state);
        channel_->registerValue("joint_pos", &buf_.joint_pos);
        channel_->registerValue("joint_vel", &buf_.joint_vel);
        channel_->registerValue("feedback_torque", &buf_.feedback_torque);
        channel_->registerValue("root_pos_w", &buf_.root_pos_w);
        channel_->registerValue("root_quat_w", &buf_.root_quat_w);
        channel_->registerValue("root_lin_vel_b", &buf_.root_lin_vel_b);
        channel_->registerValue("root_ang_vel_b", &buf_.root_ang_vel_b);
        channel_->registerValue("dof_targets", &buf_.dof_targets);
        channel_->registerValue("obs", &buf_.obs);
        channel_->registerValue("action", &buf_.action);
    }

    // Copy a sample into the registered buffers and trigger a snapshot.
    void log(const DataLogSample &sample)
    {
        buf_ = sample;
        if (channel_->takeSnapshot())
        {
            ++snapshots_;
        }
    }

    // Flush and close the MCAP file. Call on shutdown.
    void stop() { sink_->stopRecording(); }

    // Number of snapshots written so far (debugging aid).
    uint64_t snapshots() const { return snapshots_; }

private:
    std::shared_ptr<DataTamer::MCAPSink> sink_;
    std::shared_ptr<DataTamer::LogChannel> channel_;
    DataLogSample buf_;
    uint64_t snapshots_ = 0;
};

} // namespace booster_deploy
