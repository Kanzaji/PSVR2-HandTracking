# Copyright 2018-2022, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

# PSVR2 Hand Tracking project is based on https://github.com/korejan/mercury_steamvr_driver,
# which is a fork of https://github.com/moshimeow/mercury_steamvr_driver,
# which is based on the Monado project that contains the Mercury AI for Hand Tracking.
# This is already a mess...


cmake_minimum_required(VERSION 3.10.2)
project(PSVR2HandTracking VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 26)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT MSVC)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic -Wall -Wextra -Wno-unused-parameter -Werror=incompatible-pointer-types")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")
endif()

# Default to PIC code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MONADO_SRC_XRT "${CMAKE_CURRENT_SOURCE_DIR}/monado/src/xrt")
set(MONADO_SRC_EXTERNAL "${CMAKE_CURRENT_SOURCE_DIR}/monado/src/external")

### Set configuration options for Monado
set(XRT_HAVE_JPEG OFF) # No we don't
set(ZLIB_FOUND OFF) # No we didn't

set(XRT_FEATURE_COMPOSITOR_MAIN OFF) # we are not using this
set(XRT_FEATURE_COMPOSITOR_NULL ON) # Whatever
set(XRT_FEATURE_OPENXR OFF) # we need to replace this to pull together a different combination of modules
set(XRT_FEATURE_IPC OFF)
set(XRT_FEATURE_RENDERDOC OFF)
set(XRT_FEATURE_SERVICE OFF)
set(XRT_FEATURE_SERVICE_SYSTEMD OFF)
set(XRT_FEATURE_STEAMVR_PLUGIN OFF) # for now. we'll see what transpires
set(XRT_FEATURE_TRACING OFF) #ditto
set(XRT_FEATURE_SLAM OFF) #Not using it

set(XRT_MODULE_AUX_VIVE ON)
set(XRT_MODULE_MERCURY_HANDTRACKING ON)
# set(XRT_FEATURE_PROBER OFF)
set(XRT_MODULE_MONADO_CLI OFF)

set(XRT_HAVE_D3D11 OFF)
set(XRT_HAVE_D3D12 OFF)

# No drivers!
set(XRT_BUILD_DRIVER_ANDROID OFF)
set(XRT_BUILD_DRIVER_ARDUINO OFF)
set(XRT_BUILD_DRIVER_DAYDREAM OFF)
set(XRT_BUILD_DRIVER_DEPTHAI OFF)
set(XRT_BUILD_DRIVER_EUROC OFF)
set(XRT_BUILD_DRIVER_HANDTRACKING OFF)
set(XRT_BUILD_DRIVER_TWRAP OFF)
set(XRT_BUILD_DRIVER_HDK OFF)
set(XRT_BUILD_DRIVER_HYDRA OFF)
set(XRT_BUILD_DRIVER_ILLIXR OFF)
set(XRT_BUILD_DRIVER_NS OFF)
set(XRT_BUILD_DRIVER_OHMD OFF)
set(XRT_BUILD_DRIVER_OPENGLOVES OFF)
set(XRT_BUILD_DRIVER_PSMV OFF)
set(XRT_BUILD_DRIVER_PSVR OFF)
set(XRT_BUILD_DRIVER_QWERTY OFF)
set(XRT_BUILD_DRIVER_REALSENSE OFF)
set(XRT_BUILD_DRIVER_REMOTE OFF)
set(XRT_BUILD_DRIVER_RIFT_S OFF)
set(XRT_BUILD_DRIVER_SURVIVE OFF)
set(XRT_BUILD_DRIVER_ULV2 OFF)
set(XRT_BUILD_DRIVER_VF OFF)
set(XRT_BUILD_DRIVER_VIVE OFF)
set(XRT_BUILD_DRIVER_WMR OFF)
set(XRT_BUILD_DRIVER_SIMULAVR OFF)
set(XRT_BUILD_DRIVER_RIFT ON)

# There is no clean way to disable the prober/builders (as far as I can see?)
# So we enable one driver so that the legacy builder does not die
set(XRT_BUILD_DRIVER_SIMULATED ON)

set(XRT_BUILD_SAMPLES OFF)

set(BUILD_TESTING OFF)
set(BUILD_DOC OFF)

# Debug GUI Stuff
set(XRT_FEATURE_WINDOW_PEEK ON)
set(XRT_MODULE_MONADO_GUI ON)
set(XRT_DEBUG_GUI ON)
set(XRT_FEATURE_CLIENT_DEBUG_GUI ON)
set(FEATURE_OPENXR_DEBUG_UTILS ON)

# Make sure we have pretty colours
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)

#find_package(
#	OpenCV
#	REQUIRED
#	COMPONENTS
#		core
#		#calib3d
#		#highgui
#		#imgproc
#		# imgcodecs
#		#features2d
#		video
#		CONFIG
#	)
#find_package(
#	openvr_api
#	MODULE
#)

# Configure Monado

set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/deps/OpenCV/build")
set(ONNXRuntime_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}\\deps\\ONNXRuntime\\runtimes\\win-x64\\native\\onnxruntime.lib")
set(ONNXRuntime_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}\\deps\\ONNXRuntime\\build\\native\\include")
set(SDL2_DIR "${CMAKE_CURRENT_SOURCE_DIR}\\deps\\SDL2-2.32.10\\cmake")

list(APPEND CMAKE_MODULE_PATH
	"${CMAKE_SOURCE_DIR}/cmake"
	"${CMAKE_SOURCE_DIR}/monado/cmake"
	"${CMAKE_SOURCE_DIR}/monado/cmake/sanitizers"
)

add_subdirectory(monado)

add_subdirectory(src)
add_subdirectory(attic/cs_test)

# NOTE: If you disable anything in here or need to remove a library, you need to nuke the cmake build dir. It causes the project to not disable some stuff it enabled and fucks everything.
# Build notes
# You need opencv_world4120d.dll from deps/OpenCV/x64/vc16/bin in the root of the driver for the subprocess to boot.
# You also need the ONNXRuntime dll from deps.
# You need to build both driver_mercury and tracking_subprocess
# You need to manually register the driver to SteamVR
# Driver currently fails to shutdown properly and hangs SteamVR, requires manual killing of the vrserver.exe
# the subprocess fails to launch for whatever frickin reason (probably wrong working directory) - needs manual launching
# Subprocess is setup for debugging, manually set rootpath to the one found in SteamVR Web Console and manually set the port so it doesn't crash lol.
# I am doing this via debug with breakpoint on the port setting.