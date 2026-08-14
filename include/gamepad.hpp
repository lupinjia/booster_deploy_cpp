#pragma once

// Gamepad abstraction adapted for the booster RemoteControllerState message.
// The booster simulator publishes the remote controller state on
// rt/remote_controller_state; this class consumes it and provides the same
// button/axis interface as go2_deploy's gamepad.

#include <cmath>
#include <cstdint>

#include <booster/idl/b1/RemoteControllerState.h>

namespace booster_deploy
{

class Button
{
public:
    Button() = default;

    void update(bool state)
    {
        on_press = state ? (state != pressed) : false;
        on_release = state ? false : (state != pressed);
        pressed = state;
    }

    bool pressed = false;    // sustained while held
    bool on_press = false;   // single tick on press
    bool on_release = false; // single tick on release
};

class Gamepad
{
public:
    Gamepad() = default;

    // Called at the DDS frame rate: refresh smoothed axes and raw button
    // states.  Edge detection is deferred to tick().
    void set_raw(const booster_interface::msg::RemoteControllerState &key)
    {
        lx = smooth_axis(lx, key.lx());
        ly = smooth_axis(ly, key.ly());
        rx = smooth_axis(rx, key.rx());
        ry = smooth_axis(ry, key.ry());

        raw_rb = key.rb(); raw_lb = key.lb(); raw_start = key.start();
        raw_back = key.back(); raw_rt = key.rt(); raw_lt = key.lt();
        raw_a = key.a(); raw_b = key.b(); raw_x = key.x(); raw_y = key.y();
        raw_up = key.hat_u(); raw_right = key.hat_r();
        raw_down = key.hat_d(); raw_left = key.hat_l();
    }

    // Called at the control rate: computes button edges from the latest raw
    // states so on_press/on_release are observable by the control loop.
    void tick()
    {
        rb.update(raw_rb);
        lb.update(raw_lb);
        start.update(raw_start);
        back.update(raw_back);
        rt.update(raw_rt);
        lt.update(raw_lt);
        a.update(raw_a);
        b.update(raw_b);
        x.update(raw_x);
        y.update(raw_y);
        up.update(raw_up);
        right.update(raw_right);
        down.update(raw_down);
        left.update(raw_left);
    }

    float lx = 0.f, ly = 0.f, rx = 0.f, ry = 0.f;

    float smooth = 0.03f;
    float dead_zone = 0.01f;

    Button rb, lb, start, back, rt, lt, a, b, x, y, up, right, down, left;

private:
    bool raw_rb = false, raw_lb = false, raw_start = false, raw_back = false;
    bool raw_rt = false, raw_lt = false, raw_a = false, raw_b = false;
    bool raw_x = false, raw_y = false, raw_up = false, raw_right = false;
    bool raw_down = false, raw_left = false;

    float smooth_axis(float prev, float raw)
    {
        return prev * (1 - smooth) +
               (std::fabs(raw) < dead_zone ? 0.0f : raw) * smooth;
    }
};

} // namespace booster_deploy
