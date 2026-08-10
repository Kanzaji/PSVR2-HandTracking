#include "camera.h"
#include "bcdec.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_filter_one_euro.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <vector>
//#include <Eigen/src/Core/Matrix.h>
#include "../util/u_subprocess_logging.h"
#include "math/m_mathinclude.h"
#include "tinyceres/tiny_solver.hpp"
#include "tinyceres/tiny_solver_autodiff_function.hpp"

static uint8_t lut[256];

void camera_fill_lut() {
    for (size_t i = 0; i < std::size(lut); i++) {
        const double color = static_cast<double>(i) / 255.0;

        const double transformed = powf((color - 0.065) * 4, 1.0 / 2.2);

        const uint8_t result = static_cast<uint8_t>(CLAMP((int)(transformed * 255.0), 0, 255));

        lut[i] = result;
    }
}

//TODO: Make sure 21 is good for PSVR2. Taken from Rift S.
#define STEPS 21

template <typename T>
bool
kb4_distort_func(struct t_camera_calibration *calib, const T *distortion_params, const double point[2], T *out_point) {
    const double x = point[0];
    const double y = point[1];

    const double r2 = x * x + y * y;
    const double r = sqrt(r2);

    const double fx = calib->intrinsics[0][0];
    const double fy = calib->intrinsics[1][1];

    const double cx = calib->intrinsics[0][2];
    const double cy = calib->intrinsics[1][2];

    if (r < 1e-8) {
        out_point[0] = T(fx * x + cx);
        out_point[1] = T(fy * y + cy);
        return true;
    }

    const double theta = atan(r);
    const double theta2 = theta * theta;

    const T k1 = distortion_params[0];
    const T k2 = distortion_params[1];
    const T k3 = distortion_params[2];
    const T k4 = distortion_params[3];

    T r_theta = k4 * theta2;

    r_theta += k3;
    r_theta *= theta2;
    r_theta += k2;
    r_theta *= theta2;
    r_theta += k1;
    r_theta *= theta2;
    r_theta += 1;
    r_theta *= theta;

    const T mx = x * r_theta / r;
    const T my = y * r_theta / r;

    out_point[0] = fx * mx + cx;
    out_point[1] = fy * my + cy;

    return true;
}

struct TargetPoint {
    double point[2];
    double distorted[2];
};

constexpr int N_KB4_DISTORT_PARAMS = 4;

struct DistortParamKB4CostFunctor {
    DistortParamKB4CostFunctor(struct t_camera_calibration *calib, int nSteps, TargetPoint *targetPointGrid)
        : m_calib(calib), m_nSteps(nSteps), m_targetPointGrid(targetPointGrid)
    {}

    struct t_camera_calibration *m_calib;
    int m_nSteps;
    TargetPoint *m_targetPointGrid;

    template <typename T>
    bool
    operator()(const T *const distort_params, T *residual) const {
        T out_point[2];

        for (int y_index = 0; y_index < m_nSteps; y_index++) {
            for (int x_index = 0; x_index < m_nSteps; x_index++) {
                int residual_index = 2 * (y_index * m_nSteps + x_index);
                TargetPoint *p = &m_targetPointGrid[(y_index * m_nSteps) + x_index];

                if (!kb4_distort_func<T>(m_calib, distort_params, p->point, out_point))
                    return false;

                residual[residual_index + 0] = out_point[0] - p->distorted[0];
                residual[residual_index + 1] = out_point[1] - p->distorted[1];
            }
        }

        return true;
    }
};

bool get_cam_calib(t_stereo_camera_calibration **calibRef, xrt_pose *head_in_left, SharedMemoryData &shared_memory_data) {
	t_stereo_camera_calibration *calib = nullptr;
    t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_FISHEYE_KB4);

    for (int i = 0; i < 2; i++) {
        t_camera_calibration camCalib{};
        CameraIntrinsics camIntrinsics{};
        CameraParameters camParams{};

        if (!get_distortion_config(shared_memory_data, i, camParams, camIntrinsics)) {
            return false;
        }

        camCalib.image_size_pixels.w = 1016;
        camCalib.image_size_pixels.h = 1016;
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

        TargetPoint xy[STEPS * STEPS];

        // The min/max normalized coordinates that map to the image edges (0 to W-1/H-1).
        const double X_MIN = (0.0 - camIntrinsics.cx) / camIntrinsics.fx;
        const double X_MAX = (1016 - 1.0 - camIntrinsics.cx) / camIntrinsics.fx;
        const double Y_MIN = (0.0 - camIntrinsics.cy) / camIntrinsics.fy;
        const double Y_MAX = (1016 - 1.0 - camIntrinsics.cy) / camIntrinsics.fy;

        const double X_RANGE = X_MAX - X_MIN;
        const double Y_RANGE = Y_MAX - Y_MIN;

        for (int y_index = 0; y_index < STEPS; y_index++) {
            for (int x_index = 0; x_index < STEPS; x_index++) {

                double norm_x = X_MIN + (x_index / static_cast<double>((STEPS - 1))) * X_RANGE;
                double norm_y = Y_MIN + (y_index / static_cast<double>((STEPS - 1))) * Y_RANGE;

                TargetPoint *p = &xy[(y_index * STEPS) + x_index];

                p->point[0] = norm_x;
                p->point[1] = norm_y;

                DirectX::XMFLOAT2 distorted_norm = get_distorted_point(norm_x * 1.0, norm_y * 1.0, camParams);

                p->distorted[0] = distorted_norm.x * camIntrinsics.fx + camIntrinsics.cx;
                p->distorted[1] = distorted_norm.y * camIntrinsics.fy + camIntrinsics.cy;
            }
        }

        Eigen::Matrix<double, N_KB4_DISTORT_PARAMS, 1> kb4_distort_params{0, 0, 0, 0};

        using AutoDiffDistortParamKB4Function = ceres::TinySolverAutoDiffFunction<DistortParamKB4CostFunctor, 2 * STEPS * STEPS, N_KB4_DISTORT_PARAMS>;
        DistortParamKB4CostFunctor distort_param_kb4_functor(&camCalib, STEPS, xy);
        AutoDiffDistortParamKB4Function func(distort_param_kb4_functor);

        ceres::TinySolver<AutoDiffDistortParamKB4Function> solver;
        solver.Solve(func, &kb4_distort_params);

        camCalib.kb4.k1 = kb4_distort_params[0];
        camCalib.kb4.k2 = kb4_distort_params[1];
        camCalib.kb4.k3 = kb4_distort_params[2];
        camCalib.kb4.k4 = kb4_distort_params[3];

        U_SP_LOG_I("Curve-Fitting values for KB4 Distortion, Camera %d", i);
        U_SP_LOG_I("KB4.K1 => %f", kb4_distort_params[0]);
        U_SP_LOG_I("KB4.K2 => %f", kb4_distort_params[1]);
        U_SP_LOG_I("KB4.K3 => %f", kb4_distort_params[2]);
        U_SP_LOG_I("KB4.K4 => %f", kb4_distort_params[3]);

        calib->view[i] = camCalib;
    }

    // Left Camera pos from ROOT
    // Relative Pos: -0,055, 0,02589577, 0,02127628
    // Relative Quat: 0,1294095, -0,1294096, -0,01703709, 0,9829631

    // Right Camera pos from ROOT
    // Relative Pos: 0,0239805, 0,02589577, 0,02127628
    // Relative Quat: 0,1288261, 0,1328437, 0,01347646, 0,9826367

    // Right Cam to Left Cam pos (Right camera 0 0 0 -> Left Camera \/)
    // Relative Pos: -0,0761642, -0,0006115109, -0,02089401
    // Relative Quat: 0,001050473, -0,2616819, 0,003874446, 0,965146

    // Left Cam to Right Cam pos (Left camera 0 0 0 -> Right Camera \/)
    // Relative Pos: 0,0762893, 3,49246E-10, -0,02044167
    // Relative Quat: -0,001050473, 0,2616819, -0,003874446, 0,965146

    // Those are unity values, require conversions:
    // Position: Flip the Z axis. (x,y,z)→(x,y,−z).
    // Rotation (Quaternion): Mirroring the Z axis in a quaternion flip requires flipping X and Y (or Z and W). The standard transform is: (x,y,z,w)→(−x,−y,z,w)

    // double rotation[] = {0.86355550243947854, 0.010547271209912352, -0.50414348084302774,
    //                        -0.00939867939933695, 0.99994421054605376, 0.0048208527230875218,
    //                        0.50416620179465443, 0.000575209052236814, 0.863606455570185};
    // Also a difference between left and right eye?
    // double translation[] = {-0.0767532932298818, 0.00043681802400829, -0.020487561118237737};

    xrt_quat camera_rot_quat = {-0.0001221, -0.2617057, -0.0038890, 0.9651399};
    xrt_matrix_3x3 camera_rot_matrix = {};
    math_matrix_3x3_from_quat(&camera_rot_quat, &camera_rot_matrix);
    double rotation[] = {
        camera_rot_matrix.v[0],
        camera_rot_matrix.v[1],
        camera_rot_matrix.v[2],
        camera_rot_matrix.v[3],
        camera_rot_matrix.v[4],
        camera_rot_matrix.v[5],
        camera_rot_matrix.v[6],
        camera_rot_matrix.v[7],
        camera_rot_matrix.v[8]
    };
    double translation[] = {-0.0762076, 0.0000654, -0.0207520};
    memcpy(calib->camera_rotation, rotation, sizeof(rotation));
    memcpy(calib->camera_translation, translation, sizeof(translation));

    //TODO: Find a proper HeadRef for PSVR2.
    // plus_x 0 0 0 plus_z 0 0 0 and headRef 0 0 0 0 0 0 makes hands show up too high in front of the HMD.

    // xrt_vec3 plus_x {0,0,0};
    // xrt_vec3 plus_z {0,0,0};
    // -z -> Away from the HMD
    // -y -> Lower from the HMD
    // -x -> Left from the HMD
    // xrt_pose headRef {0,0,0,0, -0.15,-0.5, -0.15};

    // xrt_pose headRef {
    //     -0.1294095, 0.1294096, -0.01703709, 0.9829631,
    //     -0.055, 0.02589577, -0.02127628 // Pos y is still not low enough, this might be an issue with IPC
    // };

    xrt_pose headRef {
        { 0, 0, 0, 0 },
        { 0, 0, 0 }
     };

    math_pose_invert(&headRef, head_in_left);

    // Correctly reference count.
    t_stereo_camera_calibration_reference(calibRef, calib);
    t_stereo_camera_calibration_reference(&calib, nullptr);

    camera_fill_lut();

    return true;
}

void create_frame(const uint8_t *raw_bc4, xrt_frame *&frame) {
    u_frame_create_one_off(
        XRT_FORMAT_L8,
        REQUIRED_WIDTH, // We need 1x1 aspect ratio.
        REQUIRED_HEIGHT,
        &frame);

    if (frame == nullptr) {
        xrt_frame_reference(&frame, nullptr);
        throw std::runtime_error("Failed to allocate xrt_frame.");
    }

    if (frame->size != L8_UNCOMPRESSED_SIZE) {
        xrt_frame_reference(&frame, nullptr);
        throw std::runtime_error(std::format("Size mismatch: Allocated %zu bytes, expected %zu bytes.", frame->size, L8_UNCOMPRESSED_SIZE));
    }

    frame->stereo_format = XRT_STEREO_FORMAT_NONE;
    frame->stride = REQUIRED_WIDTH;

    for (uint32_t block_y = 0; block_y < REQUIRED_HEIGHT/4; block_y++) {
        for (uint32_t block_x = 0; block_x < REQUIRED_WIDTH/4; block_x++) {
            const size_t bc4_offset = (block_y * (IMAGE_WIDTH/4) + block_x) * BCDEC_BC4_BLOCK_SIZE;

            // Calculate output destination offset (1 byte per pixel)
            // The top-left pixel of the current 4x4 block:
            // Y position: block_y * 4 rows
            // X position: block_x * 4 columns
            const size_t l8_offset = (block_y * 4 * REQUIRED_WIDTH) + (block_x * 4);

            bcdec_bc4(raw_bc4 + bc4_offset,
                      frame->data + l8_offset,
                      REQUIRED_WIDTH);
        }
    }

    // Apply Gamma Correction.
    for (size_t i2 = 0; i2 < L8_UNCOMPRESSED_SIZE; i2++)
        frame->data[i2] = lut[frame->data[i2]];
}

bool get_camera_frames(SharedMemoryData &shared_memory_data, xrt_frame* frames[], uint32_t &latest_timestamp, uint64_t &frame_time, xrt_pose* out_pose) {
    static std::vector<uint8_t> bc4_left(BC4_DATA_SIZE);
    static std::vector<uint8_t> bc4_right(BC4_DATA_SIZE);

    const uint32_t old_timestamp = latest_timestamp;
    const bool success = copy_latest_frame_data(
        shared_memory_data,
        bc4_left.data(),
        bc4_right.data(),
        BC4_DATA_SIZE,
        latest_timestamp,
        frame_time,
        out_pose
    );

    if (!success) {
        if (latest_timestamp == old_timestamp) return false;
        throw std::runtime_error("Failed to read PSVR2 shared memory in the main loop!");
    }

    std::thread left_thread(create_frame, bc4_left.data(), std::ref(frames[0]));
    std::thread right_thread(create_frame, bc4_right.data(), std::ref(frames[1]));
    left_thread.join();
    right_thread.join();

    if (frames[0] == nullptr || frames[1] == nullptr)
        throw std::runtime_error("Failed to create xrt_frames!");

    // Set the correct timestamp for the frames.
    frames[0]->timestamp = static_cast<int64_t>(frame_time);
    frames[1]->timestamp = static_cast<int64_t>(frame_time);

    return true;
}
