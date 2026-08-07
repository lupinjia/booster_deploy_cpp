#pragma once

// Quaternion / math helpers (adapted from go2_deploy conversion.hpp).
// Quaternion convention: (w, x, y, z).

#include <array>
#include <cmath>

namespace booster_deploy
{

// Gravity vector [0,0,-1] expressed in the robot base frame.
// quat is (w,x,y,z) of the base orientation in world frame.
inline std::array<float, 3> compute_projected_gravity(const std::array<float, 4> &quat)
{
    // inverse rotation (conjugate quaternion)
    float w = quat[0];
    float x = -quat[1];
    float y = -quat[2];
    float z = -quat[3];

    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;
    float w2 = w * w;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    // R^T * [0, 0, -1]
    return std::array<float, 3>{
        -2 * (xz + wy),
        -2 * (yz - wx),
        -(w2 - x2 - y2 + z2)};
}

// Quaternion (w,x,y,z) -> RPY (roll, pitch, yaw).
inline std::array<float, 3> quat_to_rpy(const std::array<float, 4> &q)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    return std::array<float, 3>{
        std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        std::asin(2 * (w * y - z * x)),
        std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
}

// RPY (roll, pitch, yaw) -> Quaternion (w,x,y,z), XYZ (fixed-axis) convention.
inline std::array<float, 4> euler_to_quat(float roll, float pitch, float yaw)
{
    float cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    float cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    float cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return std::array<float, 4>{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy};
}

inline float wrap_to_pi(float angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace booster_deploy
