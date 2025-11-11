#pragma once
#include "xrt/xrt_defines.h"

#include "os/os_time.h"
#include "openvr.h"
#include "../tracking_subprocess_protocol.hpp"

// 80 milliseconds. I'm so sorry.
#define MAX_TIME_BETWEEN_GESTURES 80 * U_TIME_1MS_IN_NS

struct decider_global_state
{
    bool snap_up = {};
    bool snap_down = {};

    bool curled[2][4] = {};

    bool switch_tracking_gesture = false;

    int64_t last_gesture_active_time = 0;
};
extern struct decider_global_state dgs;

void decide_everything_else(tracking_message &msg, xrt_pose head, bool &grip_instead_of_aim);
