# Tracking subprocess
add_library(tracking_subprocess_protocol INTERFACE
    tracking_subprocess_protocol.hpp
)

add_executable(tracking_subprocess
        tracking_subprocess.cpp
        util/get_finger_curls.hpp util/get_finger_curls.cpp
        util/everything_else_decider.hpp util/everything_else_decider.cpp
)

target_link_libraries(tracking_subprocess PRIVATE
        # Monado and Deps
        ${OpenCV_LIBRARIES}
        xrt-interfaces
        aux_util
        aux_vive
        t_ht_mercury
        hand_async
        st_gui
        target_instance_no_comp
        # Custom
        driver_util
        oxr_sdl2_debug_hack
        psvr2_integration
        tracking_subprocess_protocol
)

set_target_properties(tracking_subprocess PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/mercury/bin/win64")

# Debug Hack
add_library(oxr_sdl2_debug_hack STATIC oxr_sdl2_hack.c)
target_link_libraries(oxr_sdl2_debug_hack PRIVATE aux_util)

if(XRT_HAVE_OPENGL)
    target_link_libraries(oxr_sdl2_debug_hack PUBLIC aux_ogl)
endif()

if(XRT_HAVE_SDL2)
    target_link_libraries(oxr_sdl2_debug_hack PRIVATE st_gui xrt-external-imgui-sdl2 ${SDL2_LIBRARIES})

    if(XRT_BUILD_DRIVER_QWERTY)
        target_link_libraries(oxr_sdl2_debug_hack PRIVATE drv_qwerty drv_qwerty_includes)
    endif()
endif()
