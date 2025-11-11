#pragma once
#include "tracking_subprocess.h"
#include "tracking_subprocess_protocol.hpp"
#include "util/everything_else_decider.hpp"
#include "xrt/xrt_defines.h"

void hjs2_to_tracking_message(subprocess_state& state, xrt_hand_joint_set sets[2], xrt_pose attached_head, tracking_message& msg, int64_t tracking_ts);

int connect_to_driver(subprocess_state& state);