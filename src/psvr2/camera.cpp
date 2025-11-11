#include "camera.h"

#include "bcdec.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_filter_one_euro.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <vector>
#include <cmath>

#define UNDIST_WIDTH 1024
#define UNDIST_HEIGHT 1024

//TODO: Replace this with real undistortion stuff. This is all CPU-Based and done for testing.
struct DistortionRemapper {
    // These tables store the source (distorted) X and Y coordinates
    // for every pixel in the destination (undistorted) 1024x1024 image.
    std::vector<float> remap_x;
    std::vector<float> remap_y;
    CameraIntrinsics intrinsics{};
    CameraParameters params{};

    DistortionRemapper() : remap_x(UNDIST_WIDTH * UNDIST_HEIGHT), remap_y(UNDIST_WIDTH * UNDIST_HEIGHT) {}
};

DistortionRemapper *remapper_left = nullptr;
DistortionRemapper *remapper_right = nullptr;

void create_remap_table(DistortionRemapper *remapper, float zoomFactor) {
    if (!remapper) return;

    // Use imageWidth/imageHeight 1024 for the target undistorted image size
    const int imageWidth = UNDIST_WIDTH;
    const int imageHeight = UNDIST_HEIGHT;

    // We're iterating over the DESTINATION (undistorted) pixels (pxOut, pyOut)
    for (int j = 0; j < imageHeight; ++j) {
        for (int i = 0; i < imageWidth; ++i) {
            auto pxOut = (float)i;
            auto pyOut = (float)j;

            // 1. Convert destination pixel to normalized coordinates (relative to focal length)
            double xNorm = (pxOut - remapper->intrinsics.cx) / remapper->intrinsics.fx;
            double yNorm = (pyOut - remapper->intrinsics.cy) / remapper->intrinsics.fy;

            // 2. Use the PSVR2 distortion math to find the source (distorted) normalized coordinates
            DirectX::XMFLOAT2 distorted_norm = get_distorted_point(xNorm * zoomFactor, yNorm * zoomFactor, remapper->params);

            // 3. Convert back to source pixel coordinates (srcPx, srcPy)
            auto srcPx = static_cast<float>(distorted_norm.x * remapper->intrinsics.fx + remapper->intrinsics.cx);
            auto srcPy = static_cast<float>(distorted_norm.y * remapper->intrinsics.fy + remapper->intrinsics.cy);

            // Store the source coordinates in the remap table
            size_t index = j * imageWidth + i;
            remapper->remap_x[index] = srcPx;
            remapper->remap_y[index] = srcPy;
        }
    }
}

void apply_remap_to_frame(const DistortionRemapper* remapper, const uint8_t* raw_l8_src, uint8_t* undistorted_l8_dst) {
    if (!remapper) return;

    for (int j = 0; j < UNDIST_HEIGHT; ++j) {
        for (int i = 0; i < UNDIST_WIDTH; ++i) {
            size_t index = j * UNDIST_WIDTH + i;

            // Lookup the source pixel coordinates
            float src_x = remapper->remap_x[index];
            float src_y = remapper->remap_y[index];

            // Use Nearest Neighbor interpolation (fastest, but lowest quality)
            int x_int = (int)std::round(src_x);
            int y_int = (int)std::round(src_y);

            uint8_t pixel_value = 0; // Default to black

            // Bounds check against the source image (1024x1024 padded)
            if (x_int >= 0 && x_int < UNDIST_WIDTH && y_int >= 0 && y_int < UNDIST_HEIGHT) {
                pixel_value = raw_l8_src[y_int * UNDIST_WIDTH + x_int];
            }

            // Write to the destination (undistorted) frame
            undistorted_l8_dst[index] = pixel_value;
        }
    }
}

bool get_cam_calib(t_stereo_camera_calibration **calibRef, xrt_pose *head_in_left, SharedMemoryData &shared_memory_data) {
	t_stereo_camera_calibration *calib = nullptr;
    //TODO: Do our own undistortion.
    t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_FISHEYE_KB4);

    DistortionRemapper* remaps[] = {remapper_left = new DistortionRemapper(), remapper_right = new DistortionRemapper()};

    for (int i = 0; i < 2; i++) {
        t_camera_calibration camCalib{};
        CameraParameters camParams{};
        CameraIntrinsics camIntrinsics{};
        auto remapper = remaps[i];

        if (!get_distortion_config(shared_memory_data, i, camParams, camIntrinsics)) {
            return false;
        }

        remapper->intrinsics = camIntrinsics;
        remapper->params = camParams;
        create_remap_table(remapper, 2);

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
        // PSVR2 has different distortion. We just give it 0 as cooeficient and provide mercury with undistorted image already.
        // camCalib.kb4.k1 = camParams.coeffs[0];
        // camCalib.kb4.k2 = camParams.coeffs[1];
        // camCalib.kb4.k3 = camParams.coeffs[2];
        // camCalib.kb4.k4 = camParams.coeffs[3];
        camCalib.kb4.k1 = 0;
        camCalib.kb4.k2 = 0;
        camCalib.kb4.k3 = 0;
        camCalib.kb4.k4 = 0;
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

float get_camera_frames(SharedMemoryData &shared_memory_data, xrt_frame* frames[]) {
    float time_camera = os_monotonic_get_ns(); //TODO: Get real camera time.
    static std::vector<uint8_t> raw_bc4_left(BC4_DATA_SIZE);
    static std::vector<uint8_t> raw_bc4_right(BC4_DATA_SIZE);
    static std::vector<uint8_t> raw_l8_left(L8_UNCOMPRESSED_SIZE);
    static std::vector<uint8_t> raw_l8_right(L8_UNCOMPRESSED_SIZE);
    static std::vector<uint8_t> undistorted_l8_left(UNDIST_WIDTH * UNDIST_HEIGHT);
    static std::vector<uint8_t> undistorted_l8_right(UNDIST_WIDTH * UNDIST_HEIGHT);
    DistortionRemapper* remaps[] = {remapper_left, remapper_right};

    const bool success = copy_latest_image_buffer(
        shared_memory_data,
        raw_bc4_left.data(),
        raw_bc4_right.data(),
        BC4_DATA_SIZE);

    if (!success) throw std::runtime_error("Failed to read PSVR2 shared memory in the main loop!");

    //TODO: BC4-Decode on the GPU. We need to undistort as well, and it just takes too long.
    // When hand is visibile framerate drops to 30 fps (~30ms/frame), this is bad.

    // BC4 Decompression (It's on the CPU, it's not great, it's fast enough (sometimes)!)
    for (int i = 0; i < 2; i++) {
        uint8_t* raw_src = (i == 0) ? raw_bc4_left.data() : raw_bc4_right.data();
        uint8_t* l8_dst = (i == 0) ? raw_l8_left.data() : raw_l8_right.data();
        uint8_t* l8_und = (i == 0)? undistorted_l8_left.data() : undistorted_l8_right.data();

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
        // Undistortion.
        // apply_remap_to_frame(remaps[i], l8_dst, l8_und);
    }

    // uint8_t* raw_data[2] = {undistorted_l8_left.data(), undistorted_l8_right.data()};
    uint8_t* raw_data[2] = {raw_l8_left.data(), raw_l8_right.data()};

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

        frames[i]->timestamp = static_cast<int64_t>(time_camera);
        frames[i]->stereo_format = XRT_STEREO_FORMAT_NONE;
    }

    return time_camera;
}
