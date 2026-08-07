#pragma once

#include <array>
#include <string>
#include <vector>

#include <booster/idl/b1/LowState.h>

#include "conversion.hpp"

namespace booster_deploy
{

// K1 real-robot joint order (identical to the mjlab ONNX policy joint order).
inline std::vector<std::string> k1_joint_names()
{
    return {
        "AAHead_yaw", "Head_pitch",
        "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
        "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
        "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch",
        "Left_Ankle_Pitch", "Left_Ankle_Roll",
        "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",
        "Right_Ankle_Pitch", "Right_Ankle_Roll"};
}

class BoosterRobotInterface
{
public:
    explicit BoosterRobotInterface(const std::vector<std::string> &joint_names)
        : joint_names_(joint_names)
    {
        size_t n = joint_names_.size();
        jpos.assign(n, 0.f);
        jvel.assign(n, 0.f);
        tau.assign(n, 0.f);
        jpos_des.assign(n, 0.f);
        jvel_des.assign(n, 0.f);
        kp.assign(n, 0.f);
        kd.assign(n, 0.f);
        tau_ff.assign(n, 0.f);
    }

    void SetState(const booster_interface::msg::LowState &state)
    {
        const auto &motors = state.motor_state_serial();
        size_t n = std::min<size_t>(motors.size(), jpos.size());
        for (size_t i = 0; i < n; ++i)
        {
            jpos[i] = motors[i].q();
            jvel[i] = motors[i].dq();
            tau[i] = motors[i].tau_est();
        }

        const auto &imu = state.imu_state();
        for (int i = 0; i < 3; ++i)
        {
            rpy[i] = imu.rpy()[i];
            gyro[i] = imu.gyro()[i];
            acc[i] = imu.acc()[i];
        }
        quat = euler_to_quat(rpy[0], rpy[1], rpy[2]);
        projected_gravity = compute_projected_gravity(quat);
    }

    const std::vector<std::string> &joint_names() const { return joint_names_; }

    // observed state (real robot order)
    std::vector<float> jpos, jvel, tau;
    std::array<float, 4> quat{1.f, 0.f, 0.f, 0.f};
    std::array<float, 3> rpy{0.f, 0.f, 0.f};
    std::array<float, 3> gyro{0.f, 0.f, 0.f};
    std::array<float, 3> acc{0.f, 0.f, 0.f};
    std::array<float, 3> projected_gravity{0.f, 0.f, -1.f};

    // desired state (real robot order)
    std::vector<float> jpos_des, jvel_des, kp, kd, tau_ff;

private:
    std::vector<std::string> joint_names_;
};

} // namespace booster_deploy
