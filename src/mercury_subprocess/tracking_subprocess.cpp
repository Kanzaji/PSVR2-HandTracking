#include "xrt/xrt_defines.h"
#pragma comment(lib, "ws2_32.lib")

// Monado
#include "tracking/t_hand_tracking.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_filter_one_euro.h"
#include "math/m_space.h"
#include "os/os_time.h"
#include "hg_interface.h"

// OpenVR
#include "openvr.h"

// OpenCV
#include <opencv2/opencv.hpp>

// PSVR2
#include "../psvr2/shared_memory.h"
#include "../psvr2/distortion.h"
#include "../psvr2/camera.h"

// Current
#include "util/pinch_decider.hpp"
#include "debug/oxr_sdl2_hack.h"
#include "debug/debug.h"
#include "u_subprocess_logging.h"
#include "tracking_subprocess.h"

#include "ipc.h"

namespace xat = xrt::auxiliary::tracking;

static char* rootpath = nullptr;

bool setup_camera_and_ht(subprocess_state& state) {
    t_stereo_camera_calibration* calib = nullptr;
    xrt_pose head_in_left{};

    if (!get_cam_calib(&calib, &head_in_left, state.shared_memory_data)) return false;

    math_pose_invert(&head_in_left, &state.left_camera_in_head);

    // zero-initialized out of paranoia
    t_hand_tracking_create_info create_info = {};
    t_camera_extra_info& info = create_info.cams_info;

    info.views[0].camera_orientation = CAMERA_ORIENTATION_0;
    info.views[1].camera_orientation = CAMERA_ORIENTATION_0;

    info.views[0].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
    info.views[1].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;

    // Taken from: https://gitlab.freedesktop.org/Beyley/monado/-/blob/dev-psvr2/src/xrt/drivers/psvr2/psvr2.c?ref_type=heads
    info.views[0].boundary.circle.normalized_center.x = 0.5f;
    info.views[0].boundary.circle.normalized_center.y = 0.5f;

    info.views[1].boundary.circle.normalized_center.x = 0.5f;
    info.views[1].boundary.circle.normalized_center.y = 0.5f;

    info.views[0].boundary.circle.normalized_radius = 0.58;
    info.views[1].boundary.circle.normalized_radius = 0.58;

    char rpath[512];
    strcpy_s(rpath, rootpath);
    strcat_s(rpath, R"(\resources\internal\hand-tracking-models\)");
    state.sync = t_hand_tracking_sync_mercury_create(calib, create_info, rpath);

    // Suggestions. These are suitable for head tracking.
    for (int i = 0; i < 2; i++) {
        m_filter_euro_vec3_init(&state.vector_filters[i], FCMIN, FCMIN_D, OUR_BETA);
        m_filter_euro_quat_init(&state.quat_filters[i], FCMIN_QUAT, FCMIN_D_QUAT, OUR_BETA_QUAT);
    }
    return true;
}

xrt_pose GetPose(const vr::HmdMatrix34_t& matrix) {
    xrt_quat q{};
    xrt_vec3 v = {matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return {q, v};
}

bool check_vrserver_alive(const subprocess_state& state) {
    vr::VREvent_t event{};
    while (state.vr_system->PollNextEvent(&event, sizeof(event))) {
        // U_SP_LOG_E("Got event %d", event.eventType);
        switch (event.eventType) {
        case vr::VREvent_Quit: {
            U_SP_LOG_I("VRServer is quitting!");
            return false;
        }
        break;
        default:
            return true;
        }
    }
    return true;
}

// Errors Codes (Yes they are in the execution order lol)
// 2 -> Missing / Incorrect args
// 3 -> Failed Init SteamVR
// 4 -> Failed to read PSVR2 data.
// 5 -> WSAStartup Failed
// 6 -> Socket Failed
// 7 -> Connection Failed
int main(int argc, char** argv) {
    U_SP_LOG_I("Starting!");

    if (argc < 3) {
        U_SP_LOG_E("Invalid args. Expected at least 3 (App Path, Root Path, Port), got %d.", argc);
        return 2;
    }

    subprocess_state state = {};
    rootpath = argv[1];
    state.port = argv[2];
    if (argc >= 4) u_sp_log_set_file_path(argv[3]);
    // if (argc >= 4) return 0;
    vr::EVRInitError error;

    for (int i = 0; i < 5; i++) {
        U_SP_LOG_I("VRInit! Attempt: %d\n", i);
        state.vr_system = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Background);
        if (error == vr::VRInitError_None) break;
        U_SP_LOG_E("Error occurred. Result: %d\n", error);
        os_nanosleep(U_TIME_1MS_IN_NS * 100);
    }

    if (error != vr::VRInitError_None) {
        U_SP_LOG_E("Failed to initialize VR Subsystem after 5 attempts, aborting!");
        return 3;
    }

    os_nanosleep(U_TIME_1S_IN_NS);

    try {
        setup_shared_memory(state.shared_memory_data);
    } catch (const std::runtime_error& e) {
        U_SP_LOG_E("Failed to read PSVR2 shared memory! Is PSVR2 turned on?");
        return 4;
    }

    if (const int i = connect_to_driver(state) != 0) return i;

    void* sdl2_hack;
    setup_debug(state, sdl2_hack);

    U_SP_LOG_I("Getting camera calibration...");
    // We setup the cam cal for PSVR2.
    setup_camera_and_ht(state);

    uint32_t latest_timestamp = 0;
    while (state.running) {
        if (!check_vrserver_alive(state)) break;

        xrt_frame* frames[2] = {nullptr, nullptr};
        const auto time_now = static_cast<float>(os_monotonic_get_ns());
        uint64_t frame_time = os_monotonic_get_ns();
        std::string failure_reason = "unknown reason";
        xrt_pose cam_pose;
        try {
            // We skip the same frames.
            if (!get_camera_frames(state.shared_memory_data, frames, latest_timestamp, frame_time, &cam_pose)) continue;
        } catch (const std::runtime_error& e) {
            failure_reason = e.what();
        }

        const float time_diff = static_cast<float>(frame_time) - time_now;
        const float time_diff_s = time_diff / static_cast<float>(U_TIME_1S_IN_NS);
        vr::TrackedDevicePose_t hmd_pose{};
        state.vr_system->GetDeviceToAbsoluteTrackingPose(
            vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, time_diff_s, &hmd_pose, 1);

       
        // U_SP_LOG_I("camera pose vs hmd pose:");
        // U_SP_LOG_I("  cam pos: %f %f %f", cam_pose.position.x, cam_pose.position.y, cam_pose.position.z);
        // U_SP_LOG_I("  cam rot: %f %f %f %f", cam_pose.orientation.w, cam_pose.orientation.x, cam_pose.orientation.y, cam_pose.orientation.z);
        // U_SP_LOG_I("  hmd pos: %f %f %f", attached_hmd_pose.position.x, attached_hmd_pose.position.y, attached_hmd_pose.position.z);
        // U_SP_LOG_I("  hmd rot: %f %f %f %f", attached_hmd_pose.orientation.w, attached_hmd_pose.orientation.x, attached_hmd_pose.orientation.y, attached_hmd_pose.orientation.z);

        xrt_hand_joint_set hands[2];
        int64_t out_timestamp;
        if (frames[0] == nullptr || frames[1] == nullptr) {
            U_SP_LOG_E("Failed creating frames for Mercury. Skipping iteration. Reason: %s", failure_reason.c_str());
            continue;
        }
        t_ht_sync_process(state.sync, frames[0], frames[1], &hands[0], &hands[1], &out_timestamp);

        for (auto & frame : frames) {
            // Discard frame references that Mercury consumed.
            xrt_frame_reference(&frame, nullptr);
        }

        //TODO: Rework IPC system. Probably use shared memory like PSVR2 Driver does, would make more sense than this.
        // Honestly I'm willing to just, well, put this into the same dll as the driver and run on separate thread? No idea why this has to be a subprocess.

        tracking_message message = {};

        message.size = TMSIZE;
        message.camera_timestamp = frame_time;
        message.host_recieved_frame_timestamp = time_now;

        hjs2_to_tracking_message(state, hands, cam_pose, message, frame_time);

        // Send data to the parent process
        message.sent_at_timestamp = os_monotonic_get_ns();

        int result = send(state.connectSocket, reinterpret_cast<const char*>(&message), TMSIZE, 0);

        if (result == SOCKET_ERROR) {
            // WSAECONNABORTED = 10053 winerror.h
            U_SP_LOG_E("Error sending data: %d", WSAGetLastError());
            break;
        }
    }

    U_SP_LOG_I("Shutting down cleanly!");

    cleanup_shared_memory(state.shared_memory_data);

    vr::VR_Shutdown();

    // Close the socket and cleanup Winsock
    closesocket(state.connectSocket);
    WSACleanup();
    oxr_sdl2_hack_stop(&sdl2_hack);

    return 0;
}