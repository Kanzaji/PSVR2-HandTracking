#pragma once

#include "distortion.h"
#include <windows.h>
#include "xrt/xrt_defines.h"

struct SharedMemoryData {
    HANDLE hMapFile = nullptr;
    LPVOID pBuf = nullptr;
    HANDLE hImageEvent = nullptr;
    HANDLE hImageMutex = nullptr;
    char* imageMemBase = nullptr;
};

void setup_shared_memory(SharedMemoryData& data);
void cleanup_shared_memory(SharedMemoryData& data);

// Get distortion configuration for a given camera ID
bool get_distortion_config(SharedMemoryData& data, int cameraId, CameraParameters& params, CameraIntrinsics& intrinsics);

// Waits for a new frame, finds the latest image buffer, and copies it to the destination.
// Returns true on success, false on failure.
bool copy_latest_frame_data(SharedMemoryData& data, void* leftCameraData, void* rightCameraData, size_t cameraDataSize, uint32_t &timestamp, uint64_t &frame_time);