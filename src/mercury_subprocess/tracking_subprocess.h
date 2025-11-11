#pragma once
// Monado
#include "tracking/t_hand_tracking.h"
#include "math/m_filter_one_euro.h"

// OpenVR
#include "openvr.h"

// PSVR2
#include "../psvr2/shared_memory.h"

// Windows
#include "winsock.h"

// Current
#include "tracking_subprocess.h"
#include "tracking_subprocess_protocol.hpp"

#define FCMIN 10.0
#define FCMIN_D 9.0
#define OUR_BETA 3.0
#define OUR_BETA_PINCHED 0.5

#define FCMIN_QUAT 1.6
#define FCMIN_D_QUAT 1.5
#define OUR_BETA_QUAT 3.0
#define OUR_BETA_PINCHED_QUAT 0.2

struct subprocess_state {
    const char* port{};
    bool running = true;

    SharedMemoryData shared_memory_data;

    SOCKET connectSocket{};

    vr::IVRSystem* vr_system{};

    xrt_pose left_camera_in_head{};

    t_hand_tracking_sync* sync = nullptr;

    // Left wrist, left indpxm, right wrist, right indpxm, head.
    // struct m_filter_euro_vec3 filters[3];
    m_filter_euro_vec3 vector_filters[2]{};
    m_filter_euro_quat quat_filters[2]{};

    bool grip_instead_of_aim = false;
    bool invert{};

    xrt_vec3 aim_orientation[2]{};

    xrt_vec3 grip_orientation[2]{};
    xrt_vec3 grip_position[2]{};

    emulated_buttons_state bs[2] = {};
};