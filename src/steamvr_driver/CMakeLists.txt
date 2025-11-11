add_library(driver_mercury MODULE
		driver_factory.cpp
		device_provider.h device_provider.cpp
        mercury_device.h mercury_device.cpp
        util/win_utils.h util/win_utils.cpp
        util/path_utils.h util/path_utils.cpp
        util/bones.h util/bones.cpp
)

target_link_libraries(
        driver_mercury
        PRIVATE
        xrt-external-openvr
        aux_util
        ${OpenCV_LIBRARIES}
        xrt-interfaces
        aux_vive
        aux_math
        t_ht_mercury
        oxr_sdl2_debug_hack
        driver_util
		tracking_subprocess_protocol
)
# copied from project_northstar_openvr_driver
target_compile_definitions(driver_mercury PUBLIC -D_EXPORT)

# meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY mercury/driver.vrdrivermanifest DESTINATION ${PROJECT_BINARY_DIR}/mercury)
file(COPY mercury/resources DESTINATION ${PROJECT_BINARY_DIR}/mercury)

set(PLUGINDIR "${PROJECT_BINARY_DIR}/mercury/bin/win64")

# message("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_mercury PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

set_target_properties(driver_mercury PROPERTIES PREFIX "")

install(
        DIRECTORY "${PROJECT_BINARY_DIR}/mercury"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/share"
)

