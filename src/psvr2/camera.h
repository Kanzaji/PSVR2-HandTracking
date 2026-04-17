#pragma once

#include "hg_interface.h"
#include "shared_memory.h"

constexpr int IMAGE_WIDTH = 1016;
constexpr int IMAGE_HEIGHT = 1024;
constexpr int REQUIRED_WIDTH = 1024;
constexpr int REQUIRED_HEIGHT = 1024;
constexpr size_t BC4_DATA_SIZE = (IMAGE_HEIGHT * IMAGE_WIDTH) / 2; // BC4 is 4 bits per pixel
constexpr size_t BC4_PADDED_SIZE = (REQUIRED_HEIGHT * REQUIRED_WIDTH) / 2;
constexpr size_t L8_UNCOMPRESSED_SIZE = 1024 * 1024; // 1MB

bool get_cam_calib(t_stereo_camera_calibration **calibRef, xrt_pose *head_in_left, SharedMemoryData &shared_memory_data);

bool get_camera_frames(SharedMemoryData &shared_memory_data, xrt_frame* frames[], uint32_t &latest_timestamp, uint64_t &frame_time, xrt_pose* out_pose);