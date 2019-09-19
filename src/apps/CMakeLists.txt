add_executable(depth2x "${CMAKE_CURRENT_LIST_DIR}/depth2x/main.cpp")
target_sources(depth2x
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/batch_converter.h"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/batch_converter.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converters.h"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_bearing.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_curvature.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_flexion.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_laserscan.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_max_curve.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/depth2x/converter_scale.cpp"
    )
add_dependencies(depth2x opencv)
target_link_libraries(depth2x
    PRIVATE
    CLI11::CLI11
    taskflow::taskflow
    fmt::fmt
    sens_loc::sens_loc
    )
common_target_properties(depth2x)

configure_file(kinect_intrinsic.txt kinect_intrinsic.txt COPYONLY)

install(TARGETS depth2x RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
