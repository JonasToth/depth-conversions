add_library(batch_processing
    "${CMAKE_CURRENT_LIST_DIR}/util/batch_converter.h"
    "${CMAKE_CURRENT_LIST_DIR}/util/batch_converter.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/util/version_printer.h"
    "${CMAKE_CURRENT_LIST_DIR}/util/version_printer.cpp"
    )
target_link_libraries(batch_processing
    PUBLIC
    sens_loc::sens_loc
    PRIVATE
    taskflow::taskflow
    fmt::fmt
    )

add_library(sens_loc::batch_processing ALIAS batch_processing)

add_executable(depth2x "${CMAKE_CURRENT_LIST_DIR}/depth2x/main.cpp")
target_sources(depth2x
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converters.h"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_scale.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_scale.h"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_bearing.h.inl"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_curvature.h.inl"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_flexion.h.inl"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_laserscan.h.inl"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_max_curve.h.inl"
    )
target_include_directories(depth2x PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(depth2x
    PRIVATE
    fmt::fmt
    CLI11::CLI11
    sens_loc::sens_loc
    sens_loc::batch_processing
    )
common_target_properties(depth2x)

configure_file(kinect_intrinsic.txt kinect_intrinsic.txt COPYONLY)
install(TARGETS depth2x RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")


add_executable(depth_filter "${CMAKE_CURRENT_LIST_DIR}/depth_filter/main.cpp")
target_sources(depth_filter
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/depth_filter/batch_filter.cpp"
    )
target_include_directories(depth_filter PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(depth_filter
    PRIVATE
    fmt::fmt
    CLI11::CLI11
    sens_loc::sens_loc
    sens_loc::batch_processing
    )
common_target_properties(depth_filter)
install(TARGETS depth_filter RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
