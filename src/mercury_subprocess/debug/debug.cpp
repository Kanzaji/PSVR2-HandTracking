#include "debug.h"

#include "oxr_sdl2_hack.h"
#include "u_subprocess_logging.h"
#include "../util/everything_else_decider.hpp"

void setup_debug(subprocess_state &state, void* &sdl2_hack) {
    U_SP_LOG_I("Setting up debug...");
    oxr_sdl2_hack_create(&sdl2_hack);
    oxr_sdl2_hack_start(sdl2_hack, nullptr, nullptr);

    u_var_add_root(&state, "SteamVR driver!", 0);

    u_var_add_bool(&state, &state.running, "Running");

    u_var_add_bool(&state, &state.bs[0].a, "left.a");
    u_var_add_bool(&state, &state.bs[0].b, "left.b");
    u_var_add_bool(&state, &state.bs[0].trigger, "left.trigger");
    u_var_add_bool(&state, &state.bs[0].thumbstick_gesture, "left.thumbstick_gesture");
    u_var_add_f32(&state, &state.bs[0].thumbstick_x, "left.thumbstick_x");
    u_var_add_f32(&state, &state.bs[0].thumbstick_y, "left.thumbstick_y");

    u_var_add_f32(&state, &state.bs[0].curls[0], "left.curls[0]");
    u_var_add_f32(&state, &state.bs[0].curls[1], "left.curls[1]");
    u_var_add_f32(&state, &state.bs[0].curls[2], "left.curls[2]");
    u_var_add_f32(&state, &state.bs[0].curls[3], "left.curls[3]");
    u_var_add_f32(&state, &state.bs[0].curls[4], "left.curls[4]");

    u_var_add_bool(&state, &dgs.curled[0][0], "left.curlsed[1]");
    u_var_add_bool(&state, &dgs.curled[0][1], "left.curlsed[2]");
    u_var_add_bool(&state, &dgs.curled[0][2], "left.curlsed[3]");
    u_var_add_bool(&state, &dgs.curled[0][3], "left.curlsed[4]");

    state.grip_instead_of_aim = true;

    state.grip_orientation[0] = {60, 4, -90};
    state.grip_position[0] = {0.04, 0.01, -0.085};

    state.grip_orientation[1] = {60, -4, 90};
    state.grip_position[1] = {-0.04, 0.01, -0.085};

    state.aim_orientation[0] = {40, 15, 0};
    state.aim_orientation[1] = {40, -15, 0};

    u_var_add_vec3_f32(&state, &state.aim_orientation[0], "aim_orientation[0]");

    u_var_add_vec3_f32(&state, &state.grip_orientation[0], "grip_orientation[0]");
    u_var_add_vec3_f32(&state, &state.grip_orientation[1], "grip_orientation[1]");

    u_var_add_vec3_f32(&state, &state.grip_position[0], "grip_position[0]");
    u_var_add_vec3_f32(&state, &state.grip_position[1], "grip_position[1]");

    u_var_add_bool(&state, &state.grip_instead_of_aim, "grip instead of aim");
    u_var_add_bool(&state, &state.invert, "invert");

    u_var_add_bool(&state, &state.bs[1].a, "right.a");
    u_var_add_bool(&state, &state.bs[1].b, "right.b");
    u_var_add_bool(&state, &state.bs[1].trigger, "right.trigger");
    u_var_add_bool(&state, &state.bs[1].thumbstick_gesture, "right.thumbstick_gesture");
    u_var_add_f32(&state, &state.bs[1].thumbstick_x, "right.thumbstick_x");
    u_var_add_f32(&state, &state.bs[1].thumbstick_y, "right.thumbstick_y");

    u_var_add_f32(&state, &state.bs[1].curls[0], "right.curls[0]");
    u_var_add_f32(&state, &state.bs[1].curls[1], "right.curls[1]");
    u_var_add_f32(&state, &state.bs[1].curls[2], "right.curls[2]");
    u_var_add_f32(&state, &state.bs[1].curls[3], "right.curls[3]");
    u_var_add_f32(&state, &state.bs[1].curls[4], "right.curls[4]");
}
