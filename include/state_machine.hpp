#pragma once

// Simple finite state machine (ported from go2_deploy state_machine.hpp).
// States: DAMPING -> STAND -> CTRL.

#include <algorithm>
#include <cstddef>

#include "user_controller.hpp"

namespace booster_deploy
{

enum class STATES
{
    DAMPING = 0,
    STAND = 1,
    CTRL = 2
};

class SimpleStateMachine
{
public:
    SimpleStateMachine() : state(STATES::DAMPING) {}

    bool Stop()
    {
        state = STATES::DAMPING;
        return true;
    }

    bool Stand()
    {
        if (state == STATES::DAMPING)
        {
            state = STATES::STAND;
            standing_count = 0;
            return true;
        }
        return false;
    }

    bool Ctrl()
    {
        if (state == STATES::STAND)
        {
            state = STATES::CTRL;
            return true;
        }
        return false;
    }

    // linear interpolation from start_pos to stand_pos over standing_duration steps
    void Standing(BasicUserController &ctrl)
    {
        float percent = static_cast<float>(standing_count) / standing_duration;
        for (size_t i = 0; i < ctrl.jpos_des.size(); ++i)
        {
            ctrl.jpos_des[i] = (1 - percent) * ctrl.start_pos[i] + percent * ctrl.stand_pos[i];
        }
        standing_count = std::min(standing_count + 1, standing_duration);
    }

    STATES state;

protected:
    int standing_count = 0;
    float standing_percent = 0.0f;
    int standing_duration = 100;
};

} // namespace booster_deploy
