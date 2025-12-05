#include "ipc.h"

#include "math/m_filter_one_euro.h"
#include "math/m_space.h"
#include "util/pinch_decider.hpp"
#include "u_subprocess_logging.h"

void hjs2_to_tracking_message(subprocess_state& state, xrt_hand_joint_set sets[2], xrt_pose attached_head,
                              tracking_message& msg, int64_t tracking_ts) {
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

            xrt_pose pose = {};
            pose.position.x = grip_position.x;
            pose.position.y = grip_position.y;
            pose.position.z = grip_position.z;

            math_quat_from_euler_angles(&hello, &pose.orientation);

            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};

            xrt_pose pose2 = {};
            pose2.orientation = XRT_QUAT_IDENTITY;
            pose2.position.x = grip_position.x;
            pose2.position.y = grip_position.y;
            pose2.position.z = grip_position.z;

            m_relation_chain_push_pose(&xrc, &pose2);

            m_relation_chain_push_pose(&xrc, &pose);

            m_relation_chain_push_pose(&xrc, &wrist_global[hand_idx]);
            m_relation_chain_resolve(&xrc, &tmp);
            ap = tmp.pose;
        } else {
            xrt_pose ap_ = aim_pose(hand_idx, wrist_global[hand_idx], index_pxm_global[hand_idx],
                                    tracked[!hand_idx] ? &wrist_global[!hand_idx] : NULL, attached_head);

            xrt_vec3 vec3 = m_vec3_mul_scalar(state.aim_orientation[hand_idx], 3.14f / 180.0f);

            xrt_quat add;

            math_quat_from_euler_angles(&vec3, &add);

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
    // decide_everything_else(msg, attached_head, state.grip_instead_of_aim);
    state.bs[0] = msg.hands[0].bs;
    state.bs[1] = msg.hands[1].bs;
}

int connect_to_driver(subprocess_state& state) {
    U_SP_LOG_I("Connecting to local host with port: %s", state.port);

    // Initialize WinSock!
    // This is an "out" struct, which we won't bother with looking at.
    WSADATA wsaData;

    U_SP_LOG_I("WSAStartup...");
    // It's also supposedly fine to have multiple overlapping calls to WSAStartup and WSACleanup. Wow Windows commits some *crimes* but alrighty
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        U_SP_LOG_E("WSAStartup failed: ", iResult);
        return 5;
    }

    U_SP_LOG_I("Setting up Socket...");

    // Create a socket to connect to the parent process
    state.connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (state.connectSocket == INVALID_SOCKET) {
        U_SP_LOG_E("Error creating socket: ", WSAGetLastError());
        WSACleanup();
        return 6;
    }

    U_SP_LOG_I("Connecting to the server...");

    // Connect to the parent process
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    serverAddr.sin_port = htons(atoi(state.port));
    iResult = connect(state.connectSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr));
    if (iResult == SOCKET_ERROR) {
        U_SP_LOG_E("Error connecting to server: ", WSAGetLastError());
        closesocket(state.connectSocket);
        WSACleanup();
        return 7;
    }

    return 0;
}