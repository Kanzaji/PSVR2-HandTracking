#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

#pragma comment(lib, "ws2_32.lib")

#include <opencv2/opencv.hpp>
#include <fstream>

#include "distortion.h"
#include "tracking/t_hand_tracking.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_space.h"
#include "hg_interface.h"

#include "os/os_time.h"
#include "tracking_subprocess_protocol.hpp"
#include "openvr.h"
#include "u_subprocess_logging.h"

#include "pinch_decider.hpp"
#include "math/m_filter_one_euro.h"
#include "everything_else_decider.hpp"
#include "shared_memory.h"
#include "util/u_frame.h"
#include "../../monado/src/external/bcdec/bcdec.h"
#include "oxr_sdl2_hack.h"

#define FCMIN 10.0
#define FCMIN_D 9.0
#define OUR_BETA 3.0
#define OUR_BETA_PINCHED 0.5

#define FCMIN_QUAT 1.6
#define FCMIN_D_QUAT 1.5
#define OUR_BETA_QUAT 3.0
#define OUR_BETA_PINCHED_QUAT 0.2

namespace xat = xrt::auxiliary::tracking;

static char* rootpath = nullptr;

struct subprocess_state {
    const char* port{};
    bool running = true;

    SharedMemoryData sharedMemoryData;

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

bool psvr2_get_cam_calib(t_stereo_camera_calibration **calibRef, xrt_pose *head_in_left, subprocess_state *state) {
	t_stereo_camera_calibration *calib = nullptr;
    //TODO: Do our own undistortion.
    t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_FISHEYE_KB4);

    for (int i = 0; i < 2; i++) {
        t_camera_calibration camCalib{};
        CameraParameters camParams{};
        CameraIntrinsics camIntrinsics{};

        if (!get_distortion_config(state->sharedMemoryData, i, camParams, camIntrinsics)) {
            return false;
        }

        // The things I'm certain are correct end on the next comment. Just sayin.
        camCalib.image_size_pixels.w = 1024;
        camCalib.image_size_pixels.h = 1024;
        camCalib.intrinsics[0][0] = camIntrinsics.fx;
        camCalib.intrinsics[0][1] = 0.0f;
        camCalib.intrinsics[0][2] = camIntrinsics.cx;
        camCalib.intrinsics[1][0] = 0.0f;
        camCalib.intrinsics[1][1] = camIntrinsics.fy;
        camCalib.intrinsics[1][2] = camIntrinsics.cy;
        camCalib.intrinsics[2][0] = 0.0f;
        camCalib.intrinsics[2][1] = 0.0f;
        camCalib.intrinsics[2][2] = 1.0f;
        camCalib.distortion_model = T_DISTORTION_FISHEYE_KB4;
        // No idea which coefficients I should use here??
        // We have... 20, KB4_FISHEYE expects 4 arguments and the Index provides 4.
        // Should we calculate average from those 20 somehow or just?????
        camCalib.kb4.k1 = camParams.coeffs[0];
        camCalib.kb4.k2 = camParams.coeffs[1];
        camCalib.kb4.k3 = camParams.coeffs[2];
        camCalib.kb4.k4 = camParams.coeffs[3];
        calib->view[i] = camCalib;
    }

    // Those are real sub-mm camera rotation and translation for PSVR2, done with Monado Calibration code (Long Gone apparently)
    // HUGE Thanks to Lyris for finding those in their notes <3
    double rotation[] = {0.86355550243947854, 0.010547271209912352, -0.50414348084302774,
                           -0.00939867939933695, 0.99994421054605376, 0.0048208527230875218,
                           0.50416620179465443, 0.000575209052236814, 0.863606455570185};
    double translation[] = {-0.0767532932298818, 0.00043681802400829, -0.020487561118237737};
    memcpy(calib->camera_rotation, rotation, sizeof(rotation));
    memcpy(calib->camera_translation, translation, sizeof(translation));

    //TODO: Find a proper HeadRef for PSVR2. This is still stollen from Index.
    xrt_vec3 plus_x {-1,0,0};
    xrt_vec3 plus_z {0,-0,-1};
    // Pos Stolen from Index? No idea if correct (probably not) for PSVR2.
    xrt_pose headRef {0,0,0,0, 0,0, -0.01092000026255846};

    math_quat_from_plus_x_z(&plus_x, &plus_z, &headRef.orientation);

    math_pose_invert(&headRef, head_in_left);

    // Correctly reference count.
    t_stereo_camera_calibration_reference(calibRef, calib);
    t_stereo_camera_calibration_reference(&calib, nullptr);

    return true;
}

bool setup_camera_and_ht(subprocess_state& state) {
    t_stereo_camera_calibration* calib = nullptr;
    xrt_pose head_in_left{}; // unused???

    if (!psvr2_get_cam_calib(&calib, &head_in_left, &state)) return false;

    math_pose_invert(&head_in_left, &state.left_camera_in_head);

    // zero-initialized out of paranoia
    t_hand_tracking_create_info create_info = {};
    t_camera_extra_info& info = create_info.cams_info;

    info.views[0].camera_orientation = CAMERA_ORIENTATION_0;
    info.views[1].camera_orientation = CAMERA_ORIENTATION_0;

    info.views[0].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
    info.views[1].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;

    // No clue if those are in practice any good for PSVR2. We will see!
    info.views[0].boundary.circle.normalized_center.x = 0.5f;
    info.views[0].boundary.circle.normalized_center.y = 0.5f;

    info.views[1].boundary.circle.normalized_center.x = 0.5f;
    info.views[1].boundary.circle.normalized_center.y = 0.5f;

    info.views[0].boundary.circle.normalized_radius = 0.55;
    info.views[1].boundary.circle.normalized_radius = 0.55;

    char rpath[512];
    strcpy_s(rpath, rootpath);
    strcat_s(rpath, R"(\resources\internal\hand-tracking-models\)");
    state.sync =
        t_hand_tracking_sync_mercury_create(calib, create_info, rpath);

    // // Suggestions. These are suitable for head tracking.
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

void hjs2_to_tracking_message(subprocess_state& state, xrt_hand_joint_set sets[2], xrt_pose attached_head,
                              struct tracking_message& msg, int64_t tracking_ts) {
    bool tracked[2] = {false, false};
    xrt_pose wrist_global[2] = {};
    xrt_pose index_pxm_global[2] = {};

    // msg.hands[0].bs = state.bs[0];
    // msg.hands[1].bs = state.bs[1];
    // state.bs[1] = msg.hands[1].bs;

    for (int hand_idx = 0; hand_idx < 2; hand_idx++) {
        xrt_hand_joint_set& set = sets[hand_idx];

        msg.hands[hand_idx].tracked = set.is_active;
        tracked[hand_idx] = set.is_active;
        if (!msg.hands[hand_idx].tracked) {
            state.vector_filters[hand_idx].base.have_prev_y = false;
            state.quat_filters[hand_idx].base.have_prev_y = false;
            // m_filter_euro_vec3_init(&state.vector_filters[hand_idx], FCMIN, FCMIN_D, OUR_BETA);
            // m_filter_euro_quat_init(&state.quat_filters[hand_idx], FCMIN_QUAT, FCMIN_D_QUAT, OUR_BETA_QUAT);
            continue;
        }

        trigger_decide(set, &state.bs[hand_idx].trigger);
        msg.hands[hand_idx].bs = state.bs[hand_idx];

        xrt_space_relation wrist = set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation;
        xrt_space_relation index_pxm = set.values.hand_joint_set_default[XRT_HAND_JOINT_INDEX_PROXIMAL].relation;

        {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &wrist);
            // TODO ADD HEAD OFFSET HERE
            m_relation_chain_push_pose(&xrc, &state.left_camera_in_head);
            m_relation_chain_push_pose(&xrc, &attached_head);

            m_relation_chain_resolve(&xrc, &tmp);

            wrist_global[hand_idx] = tmp.pose;
        }

        {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &index_pxm);
            // TODO ADD HEAD OFFSET HERE
            m_relation_chain_push_pose(&xrc, &state.left_camera_in_head);
            m_relation_chain_push_pose(&xrc, &attached_head);

            m_relation_chain_resolve(&xrc, &tmp);

            index_pxm_global[hand_idx] = tmp.pose;
        }
    }

    for (int hand_idx = 0; hand_idx < 2; hand_idx++) {
        if (!tracked[hand_idx]) {
            continue;
        }
        xrt_hand_joint_set& set = sets[hand_idx];

        xrt_pose ap{};

        if (state.grip_instead_of_aim) {
            xrt_vec3 hello = {};
            xrt_vec3& grip_position = state.grip_position[hand_idx];

            hello.x = state.grip_orientation[hand_idx].x * (3.14 / 180);
            hello.y = state.grip_orientation[hand_idx].y * (3.14 / 180);
            hello.z = state.grip_orientation[hand_idx].z * (3.14 / 180);

            xrt_pose meow = {};
            meow.position.x = grip_position.x;
            meow.position.y = grip_position.y;
            meow.position.z = grip_position.z;

            math_quat_from_euler_angles(&hello, &meow.orientation);

            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};

            xrt_pose meow2 = {};
            meow2.orientation = XRT_QUAT_IDENTITY;
            meow2.position.x = grip_position.x;
            meow2.position.y = grip_position.y;
            meow2.position.z = grip_position.z;

            m_relation_chain_push_pose(&xrc, &meow2);

            m_relation_chain_push_pose(&xrc, &meow);

            m_relation_chain_push_pose(&xrc, &wrist_global[hand_idx]);
            m_relation_chain_resolve(&xrc, &tmp);
            ap = tmp.pose;
        } else {
            xrt_pose ap_ = aim_pose(hand_idx, wrist_global[hand_idx], index_pxm_global[hand_idx],
                                    tracked[!hand_idx] ? &wrist_global[!hand_idx] : NULL, attached_head);

            xrt_vec3 meow = m_vec3_mul_scalar(state.aim_orientation[hand_idx], 3.14f / 180.0f);

            xrt_quat add;

            math_quat_from_euler_angles(&meow, &add);

            math_quat_rotate(&ap_.orientation, &add, &ap_.orientation);

            if (msg.hands[hand_idx].bs.trigger) {
                // Too low
                // const float mul = 0.000001;

                // Also too low
                // const float mul = 0.0001;

                // Also too low - these all feel the same and result in ~no movement
                // const float mul = 0.001;

                // This does result in some movement
                // const float mul = 0.01;

                // OK this is good and feels purposeful and generally helps interactions
                const float mul = 0.05;

                state.quat_filters[hand_idx].base.fc_min = FCMIN_QUAT * mul;
                state.quat_filters[hand_idx].base.fc_min_d = FCMIN_D_QUAT * mul;
                state.quat_filters[hand_idx].base.beta = OUR_BETA_PINCHED_QUAT;

                state.vector_filters[hand_idx].base.fc_min = FCMIN * mul;
                state.vector_filters[hand_idx].base.fc_min_d = FCMIN_D * mul;
                state.quat_filters[hand_idx].base.beta = OUR_BETA_PINCHED;
            } else {
                state.quat_filters[hand_idx].base.fc_min = FCMIN_QUAT;
                state.quat_filters[hand_idx].base.fc_min_d = FCMIN_D_QUAT;
                state.quat_filters[hand_idx].base.beta = OUR_BETA_QUAT;

                state.vector_filters[hand_idx].base.fc_min = FCMIN;
                state.vector_filters[hand_idx].base.fc_min_d = FCMIN_D;
                state.quat_filters[hand_idx].base.beta = OUR_BETA;
            }

            m_filter_euro_quat_run(&state.quat_filters[hand_idx], tracking_ts, &ap_.orientation, &ap.orientation);
            m_filter_euro_vec3_run(&state.vector_filters[hand_idx], tracking_ts, &ap_.position, &ap.position);
        }

        msg.hands[hand_idx].pose_raw = ap;
        msg.hands[hand_idx].wrist = wrist_global[hand_idx];
        for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++) {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &set.values.hand_joint_set_default[i].relation);
            m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &wrist_global[hand_idx]);
            m_relation_chain_resolve(&xrc, &tmp);

            msg.hands[hand_idx].fingers_relative[i] = tmp.pose;
        }
    }
    decide_everything_else(msg, attached_head, state.grip_instead_of_aim);
    state.bs[0] = msg.hands[0].bs;
    state.bs[1] = msg.hands[1].bs;
}

bool check_vrserver_alive(subprocess_state& state) {
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
// 3 -> Failed to read PSVR2 data.
// 4 -> Failed Init SteamVR
// 5 -> WSAStartup Failed
// 6 -> Socket Failed
// 7 -> Connection Failed
int main(int argc, char** argv) {
    U_SP_LOG_I("Starting!");

    if (argc < 3) {
        U_SP_LOG_E("Invalid args. Expected at least 3 (App Path, Root Path, Port), got %d.", argc);
        // return 2;
    }
    subprocess_state state = {};

    try {
        setup_shared_memory(state.sharedMemoryData);
    } catch (const std::runtime_error& e) {
        U_SP_LOG_E("Failed to read PSVR2 shared memory! Is PSVR2 turned on?");
        return 3;
    }

    rootpath = argv[1];
    // state.port = argv[2];

    if (argc >= 4) u_sp_log_set_file_path(argv[3]);

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
        return 4;
    }

    // U_SP_LOG_I("Connecting to local host with port: %s", state.port);
    //
    // // Initialize WinSock!
    // // This is an "out" struct, which we won't bother with looking at.
    // WSADATA wsaData;
    //
    // U_SP_LOG_I("WSAStartup...");
    // // It's also supposedly fine to have multiple overlapping calls to WSAStartup and WSACleanup. Wow Windows commits some *crimes* but alrighty
    // int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    // if (iResult != 0) {
    //     U_SP_LOG_E("WSAStartup failed: ", iResult);
    //     return 5;
    // }
    //
    // U_SP_LOG_I("Setting up Socket...");
    //
    // // Create a socket to connect to the parent process
    // state.connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    // if (state.connectSocket == INVALID_SOCKET) {
    //     U_SP_LOG_E("Error creating socket: ", WSAGetLastError());
    //     WSACleanup();
    //     return 6;
    // }
    //
    // U_SP_LOG_I("Connecting to the server...");
    //
    // // Connect to the parent process
    // sockaddr_in serverAddr{};
    // serverAddr.sin_family = AF_INET;
    // serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    //
    // serverAddr.sin_port = htons(atoi(state.port));
    // iResult = connect(state.connectSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr));
    // if (iResult == SOCKET_ERROR) {
    //     U_SP_LOG_E("Error connecting to server: ", WSAGetLastError());
    //     closesocket(state.connectSocket);
    //     WSACleanup();
    //     return 7;
    // }

    U_SP_LOG_I("Setting up debug [Part 1]");
    void* sdl2_hack;
    oxr_sdl2_hack_create(&sdl2_hack);

    U_SP_LOG_I("Geting camera calibration...");

    // We setup the cam cal for PSVR2.
    setup_camera_and_ht(state);

    U_SP_LOG_I("Setting up debug [Part 2]");
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

    const int IMAGE_WIDTH = 1016;
    const int IMAGE_HEIGHT = 1024;
    const int REQUIRED_WIDTH = 1024;
    const int REQUIRED_HEIGHT = 1024;
    const size_t BC4_DATA_SIZE = (IMAGE_HEIGHT * IMAGE_WIDTH) / 2; // BC4 is 4 bits per pixel
    const size_t BC4_PADDED_SIZE = (REQUIRED_HEIGHT * REQUIRED_WIDTH) / 2;
    const size_t L8_UNCOMPRESSED_SIZE = 1024 * 1024; // 1MB

    while (state.running) {
        if (!check_vrserver_alive(state)) break;

        static std::vector<uint8_t> raw_bc4_left(BC4_DATA_SIZE);
        static std::vector<uint8_t> raw_bc4_right(BC4_DATA_SIZE);
        static std::vector<uint8_t> raw_l8_left(L8_UNCOMPRESSED_SIZE);
        static std::vector<uint8_t> raw_l8_right(L8_UNCOMPRESSED_SIZE);

        bool success = copy_latest_image_buffer(
            state.sharedMemoryData,
            raw_bc4_left.data(),
            raw_bc4_right.data(),
            BC4_DATA_SIZE);

        if (!success) {
            U_SP_LOG_E("Failed to read PSVR2 shared memory in the main loop!");
            os_nanosleep(U_TIME_1MS_IN_NS * 10);
            continue;
        }

        // BC4 Decompression (It's on the CPU, it's not great, it's fast enough!)
        // If I end up needing to undistort it as well, I will move this to the GPU.
        for (int i = 0; i < 2; i++) {
            uint8_t* raw_src = (i == 0) ? raw_bc4_left.data() : raw_bc4_right.data();
            uint8_t* l8_dst = (i == 0) ? raw_l8_left.data() : raw_l8_right.data();

            memset(l8_dst, 0, 1024 * REQUIRED_HEIGHT);
            // Loop through 254 rows of blocks
            for (uint32_t block_y = 0; block_y < IMAGE_HEIGHT/4; block_y++) {
                // Loop through 256 columns of blocks
                for (uint32_t block_x = 0; block_x < IMAGE_WIDTH/4; block_x++) {
                    size_t bc4_offset = (block_y * (REQUIRED_WIDTH/4) + block_x) * BCDEC_BC4_BLOCK_SIZE;

                    // Calculate output destination offset (1 byte per pixel)
                    // The top-left pixel of the current 4x4 block:
                    // Y position: block_y * 4 rows
                    // X position: block_x * 4 columns
                    size_t l8_offset = (block_y * 4 * 1024) + (block_x * 4);

                    // Let Monado decode the BC4 Block.
                    bcdec_bc4(raw_src + bc4_offset,
                              l8_dst + l8_offset,
                              1024);
                }
            }
        }

        uint64_t time_now_uint = os_monotonic_get_ns();
        auto time_now = static_cast<float>(time_now_uint);
        auto time_camera = static_cast<float>(time_now_uint); //TODO: Get actual cam time data.
        // double time_camera = state.cap.get(cv::CAP_PROP_POS_MSEC) * 1e6;

        float time_ratio = time_now / time_camera; // This is used for what?

        float time_diff = time_camera - time_now;

        float time_diff_s = time_diff / static_cast<float>(U_TIME_1S_IN_NS);

        vr::TrackedDevicePose_t hmd_pose{};

        state.vr_system->GetDeviceToAbsoluteTrackingPose(
            vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, time_diff_s, &hmd_pose, 1);

        xrt_pose attached_hmd_pose = GetPose(hmd_pose.mDeviceToAbsoluteTracking);
        xrt_frame* frames[2] = {nullptr, nullptr};
        uint8_t* raw_data[2] = {raw_l8_left.data(), raw_l8_right.data()};

        for (int i = 0; i < 2; i++) {
            u_frame_create_one_off(
                XRT_FORMAT_L8,
                REQUIRED_WIDTH, // We need 1x1 aspect ratio.
                REQUIRED_HEIGHT,
                &frames[i]);

            if (frames[i] == nullptr) {
                U_SP_LOG_E("Failed to allocate xrt_frame.");
                xrt_frame_reference(&frames[0], nullptr);
                xrt_frame_reference(&frames[1], nullptr);
                break;
            }

            if (frames[i]->size != L8_UNCOMPRESSED_SIZE) {
                U_SP_LOG_E("Size mismatch: Allocated %zu bytes, expected %zu bytes.", frames[i]->size, BC4_PADDED_SIZE);
                xrt_frame_reference(&frames[0], nullptr);
                xrt_frame_reference(&frames[1], nullptr);
                break;
            }
            memcpy(frames[i]->data, raw_data[i], L8_UNCOMPRESSED_SIZE);
            // memset(frames[i]->data, 0, frames[i]->size);

            frames[i]->timestamp = static_cast<int64_t>(time_camera);
            frames[i]->stereo_format = XRT_STEREO_FORMAT_NONE;
        }


        xrt_hand_joint_set hands[2];
        int64_t out_timestamp;
        if (frames[0] == nullptr || frames[1] == nullptr) {
            U_SP_LOG_E("Failed creating frames for Mercury. Skipping iteration.");
            continue;
        }
        t_ht_sync_process(state.sync, frames[0], frames[1], &hands[0], &hands[1], &out_timestamp);

        for (auto & frame : frames) {
            // Discard frame references that Mercury consumed.
            xrt_frame_reference(&frame, nullptr);
        }

        // tracking_message message = {};
        //
        // message.size = TMSIZE;
        // message.camera_timestamp = static_cast<int64_t>(time_camera);
        // message.host_recieved_frame_timestamp = static_cast<int64_t>(time_now_uint);
        //
        // hjs2_to_tracking_message(state, hands, attached_hmd_pose, message, static_cast<int64_t>(time_camera));
        //
        // // Send data to the parent process
        // message.sent_at_timestamp = os_monotonic_get_ns();
        //
        // iResult = send(state.connectSocket, reinterpret_cast<const char*>(&message), TMSIZE, 0);
        //
        // if (iResult == SOCKET_ERROR) {
        //     // WSAECONNABORTED = 10053 winerror.h
        //     U_SP_LOG_E("Error sending data: %d", WSAGetLastError());
        //     break;
        // }
    }

    U_SP_LOG_I("Shutting down cleanly!");

    cleanup_shared_memory(state.sharedMemoryData);

    vr::VR_Shutdown();

    // Close the socket and cleanup Winsock
    closesocket(state.connectSocket);
    WSACleanup();
    oxr_sdl2_hack_stop(&sdl2_hack);

    return 0;
}
