add_library(psvr2_integration STATIC
    distortion.cpp distortion.h
    shared_memory.cpp shared_memory.h
    camera.cpp camera.h
)

target_link_libraries(psvr2_integration PRIVATE
        # Monado and Deps
        ${OpenCV_LIBRARIES}
        xrt-interfaces
        aux_util
        aux_vive
        t_ht_mercury
        hand_async
        st_gui
        target_instance_no_comp
)
