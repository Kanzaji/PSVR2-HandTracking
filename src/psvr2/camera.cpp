#include "camera.h"

#include "bcdec.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_filter_one_euro.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <vector>

static uint8_t lut[256];

void camera_fill_lut() {
    for (size_t i = 0; i < std::size(lut); i++) {
        const double color = static_cast<double>(i) / 255.0;

        const double transformed = powf((color - 0.065) * 4, 1.0 / 2.2);

        const auto result = static_cast<uint8_t>(CLAMP((int)(transformed * 255.0), 0, 255));

        lut[i] = result;
    }
}

bool get_cam_calib(t_stereo_camera_calibration **calibRef, xrt_pose *head_in_left, SharedMemoryData &shared_memory_data) {
	t_stereo_camera_calibration *calib = nullptr;
    t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_FISHEYE_KB4);

    for (int i = 0; i < 2; i++) {
        t_camera_calibration camCalib{};
        CameraParameters camParams{};
        CameraIntrinsics camIntrinsics{};

        if (!get_distortion_config(shared_memory_data, i, camParams, camIntrinsics)) {
            return false;
        }

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
        //TODO: Curve-fit PSVR2 to this.
        // Currently taken from https://gitlab.freedesktop.org/Beyley/monado/-/blob/dev-psvr2/src/xrt/drivers/psvr2/psvr2.c?ref_type=heads
        camCalib.kb4.k1 = 0.027694760153821763,
        camCalib.kb4.k2 = -0.015417764941581483,
        camCalib.kb4.k3 = 0.0068598424886571066,
        camCalib.kb4.k4 = -0.00171888994824757,

        calib->view[i] = camCalib;
    }

    // Those are real sub-mm camera rotation and translation for PSVR2, done with Monado Calibration code (Long Gone apparently)
    // HUGE Thanks to Lyris for finding those in their notes <3
    // Found source! https://gitlab.freedesktop.org/Beyley/monado/-/blob/dev-psvr2/src/xrt/drivers/psvr2/psvr2.c
    double rotation[] = {0.86355550243947854, 0.010547271209912352, -0.50414348084302774,
                           -0.00939867939933695, 0.99994421054605376, 0.0048208527230875218,
                           0.50416620179465443, 0.000575209052236814, 0.863606455570185};
    double translation[] = {-0.0767532932298818, 0.00043681802400829, -0.020487561118237737};
    memcpy(calib->camera_rotation, rotation, sizeof(rotation));
    memcpy(calib->camera_translation, translation, sizeof(translation));

    //TODO: Find a proper HeadRef for PSVR2. This is still stollen from Index.
    xrt_vec3 plus_x {-1,0,0};
    xrt_vec3 plus_z {0,-0,-1};
    xrt_pose headRef {0,0,0,0, 0,0, -0.01092000026255846};

    math_quat_from_plus_x_z(&plus_x, &plus_z, &headRef.orientation);

    math_pose_invert(&headRef, head_in_left);

    // Correctly reference count.
    t_stereo_camera_calibration_reference(calibRef, calib);
    t_stereo_camera_calibration_reference(&calib, nullptr);

    camera_fill_lut();

    return true;
}

void decode_bc4_frame(uint8_t *raw_bc4, uint8_t * dst_l8) {
    //TODO: BC4-Decode on the GPU.

    // BC4 Decompression (It's on the CPU, it's not great, it's fast enough (sometimes)!)
    memset(dst_l8, 0, REQUIRED_WIDTH * REQUIRED_HEIGHT);
    // Loop through 254 rows of blocks
    for (uint32_t block_y = 0; block_y < IMAGE_HEIGHT/4; block_y++) {
        // Loop through 256 columns of blocks
        for (uint32_t block_x = 0; block_x < IMAGE_WIDTH/4; block_x++) {
            const size_t bc4_offset = (block_y * (REQUIRED_WIDTH/4) + block_x) * BCDEC_BC4_BLOCK_SIZE;

            // Calculate output destination offset (1 byte per pixel)
            // The top-left pixel of the current 4x4 block:
            // Y position: block_y * 4 rows
            // X position: block_x * 4 columns
            const size_t l8_offset = (block_y * 4 * 1024) + (block_x * 4);

            // Let Monado decode the BC4 Block.
            bcdec_bc4(raw_bc4 + bc4_offset,
                      dst_l8 + l8_offset,
                      1024);
        }
    }

    // Apply Gamma Correction.
    for (size_t i2 = 0; i2 < L8_UNCOMPRESSED_SIZE; i2++)
        dst_l8[i2] = lut[dst_l8[i2]];
}

bool get_camera_frames(SharedMemoryData &shared_memory_data, xrt_frame* frames[], uint32_t &latest_timestamp) {
    static std::vector<uint8_t> bc4_left(BC4_DATA_SIZE);
    static std::vector<uint8_t> bc4_right(BC4_DATA_SIZE);
    static std::vector<uint8_t> l8_left(L8_UNCOMPRESSED_SIZE);
    static std::vector<uint8_t> l8_right(L8_UNCOMPRESSED_SIZE);

    const uint32_t old_timestamp = latest_timestamp;
    const bool success = copy_latest_image_buffer(
        shared_memory_data,
        bc4_left.data(),
        bc4_right.data(),
        BC4_DATA_SIZE,
        latest_timestamp
    );

    if (!success) {
        if (latest_timestamp == old_timestamp) return false;
        throw std::runtime_error("Failed to read PSVR2 shared memory in the main loop!");
    }

    std::thread left_thread(decode_bc4_frame, bc4_left.data(), l8_left.data());
    std::thread right_thread(decode_bc4_frame, bc4_right.data(), l8_right.data());
    left_thread.join();
    right_thread.join();

    uint8_t* raw_data[2] = {l8_left.data(), l8_right.data()};

    for (int i = 0; i < 2; i++) {
        u_frame_create_one_off(
            XRT_FORMAT_L8,
            REQUIRED_WIDTH, // We need 1x1 aspect ratio.
            REQUIRED_HEIGHT,
            &frames[i]);

        if (frames[i] == nullptr) {
            xrt_frame_reference(&frames[0], nullptr);
            xrt_frame_reference(&frames[1], nullptr);
            throw std::runtime_error("Failed to allocate xrt_frame.");
        }

        if (frames[i]->size != L8_UNCOMPRESSED_SIZE) {
            xrt_frame_reference(&frames[0], nullptr);
            xrt_frame_reference(&frames[1], nullptr);
            throw std::runtime_error(std::format("Size mismatch: Allocated %zu bytes, expected %zu bytes.", frames[i]->size, BC4_PADDED_SIZE));
        }
        memcpy(frames[i]->data, raw_data[i], L8_UNCOMPRESSED_SIZE);
        // memset(frames[i]->data, 0, frames[i]->size);

        frames[i]->timestamp = os_monotonic_get_ns();
        frames[i]->stereo_format = XRT_STEREO_FORMAT_NONE;
    }

    return true;
}
