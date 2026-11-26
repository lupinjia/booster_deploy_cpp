// Virtual remote controller for headless integration testing.
//
// Publishes RemoteControllerState frames continuously (50Hz) on
// rt/remote_controller_state so booster_deploy_cpp can be exercised without a
// physical gamepad.  A real gamepad (via booster_mujoco) publishes at 500Hz,
// so continuous publishing is required for reliable edge detection.
//
// Scripted phases (state machine flow mirrors go2_deploy):
//   phase 1 (3s):  START button press        -> deploy starts control loop
//   phase 2 (3s):  LB + B                    -> state machine STAND
//   phase 3 (2s):  LB + A                    -> state machine CTRL (policy active)
//   phase 4 (0.5s): RT press                 -> speed gear 2 (higher max vx)
//   phase 5 (10s): ramp vx (left stick up)   -> velocity command
//   phase 6 (3s):  LB + Y                    -> STOP (damping)

#include <chrono>
#include <cstdio>
#include <thread>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/idl/b1/RemoteControllerState.h>

#define TOPIC_REMOTE "rt/remote_controller_state"

using booster::robot::ChannelFactory;
using booster::robot::ChannelPublisher;
using booster_interface::msg::RemoteControllerState;

static void set_all_false(RemoteControllerState &m)
{
    m.event(0);
    m.lx(0); m.ly(0); m.rx(0); m.ry(0);
    m.a(false); m.b(false); m.x(false); m.y(false);
    m.lb(false); m.rb(false); m.lt(false); m.rt(false);
    m.ls(false); m.rs(false); m.back(false); m.start(false);
    m.hat_u(false); m.hat_d(false); m.hat_l(false); m.hat_r(false);
    m.hat_lu(false); m.hat_ld(false); m.hat_ru(false); m.hat_rd(false);
}

int main(int argc, char **argv)
{
    std::string interface = "lo";
    if (argc >= 2) interface = argv[1];
    ChannelFactory::Instance()->Init(1, interface);

    ChannelPublisher<RemoteControllerState> pub(TOPIC_REMOTE);
    pub.InitChannel();
    std::printf("[virtual_remote] publishing on %s (interface %s)\n", TOPIC_REMOTE, interface.c_str());

    const int TICK_MS = 20;
    enum Phase { PHASE_IDLE, PHASE_START, PHASE_STAND, PHASE_CTRL, PHASE_GEAR2, PHASE_RAMP, PHASE_STOP };
    auto phase_start = std::chrono::steady_clock::now();
    Phase phase = PHASE_IDLE;

    RemoteControllerState msg;
    int ticks = 0;

    while (true)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - phase_start).count();
        switch (phase)
        {
        case PHASE_IDLE:
            if (elapsed > 12000) { phase = PHASE_START; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] START\n"); }
            break;
        case PHASE_START:
            if (elapsed > 1500) { phase = PHASE_STAND; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] LB+B (STAND)\n"); }
            break;
        case PHASE_STAND:
            if (elapsed > 3000) { phase = PHASE_CTRL; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] LB+A (CTRL)\n"); }
            break;
        case PHASE_CTRL:
            if (elapsed > 2000) { phase = PHASE_GEAR2; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] RT (speed gear 2)\n"); }
            break;
        case PHASE_GEAR2:
            if (elapsed > 500) { phase = PHASE_RAMP; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] velocity ramp\n"); }
            break;
        case PHASE_RAMP:
            if (elapsed > 10000) { phase = PHASE_STOP; phase_start = std::chrono::steady_clock::now(); std::printf("[virtual_remote] LB+Y (STOP)\n"); }
            break;
        case PHASE_STOP:
            if (elapsed > 3000) { std::printf("[virtual_remote] done\n"); return 0; }
            break;
        }

        set_all_false(msg);
        if (phase == PHASE_START) msg.start(true);
        else if (phase == PHASE_STAND) { msg.lb(true); msg.b(true); }
        else if (phase == PHASE_CTRL) { msg.lb(true); msg.a(true); }
        else if (phase == PHASE_GEAR2) { msg.rt(true); }
        else if (phase == PHASE_RAMP)
        {
            msg.ly(0.1f * (ticks % 20 + 1) / 2.f);
        }
        else if (phase == PHASE_STOP) { msg.lb(true); msg.y(true); }
        pub.Write(&msg);
        ++ticks;

        std::this_thread::sleep_for(std::chrono::milliseconds(TICK_MS));
    }
}
