# Copyright 2023, Moshi Turner
# SPDX-License-Identifier: BSL-1.0

add_library(driver_util STATIC
        CameraIdxGuesser.cpp CameraIdxGuesser.hpp
        CameraIdxGuesser_win_capture.h
        driver_log.cpp driver_log.h
        u_subprocess_logging.c u_subprocess_logging.h
)


target_include_directories(driver_util INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(driver_util PRIVATE
        ${OpenCV_LIBRARIES}
        aux_util
        xrt-external-openvr
)
