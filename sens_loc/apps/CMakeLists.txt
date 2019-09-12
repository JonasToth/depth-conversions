add_executable(depth2x "${CMAKE_CURRENT_LIST_DIR}/depth2x/main_single.cpp")
add_dependencies(depth2x opencv)
target_link_libraries(depth2x
    PRIVATE
    CLI11::CLI11
    taskflow::taskflow
    sens_loc::sens_loc
    )
common_target_properties(depth2x)

add_executable(depth2x-batch "${CMAKE_CURRENT_LIST_DIR}/depth2x/main_batch.cpp")
add_dependencies(depth2x-batch opencv)
target_link_libraries(depth2x-batch
    PRIVATE
    CLI11::CLI11
    taskflow::taskflow
    fmt::fmt
    sens_loc::sens_loc
    )
common_target_properties(depth2x-batch)

configure_file(kinect_intrinsic.txt kinect_intrinsic.txt COPYONLY)

install(TARGETS depth2x RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS depth2x-batch RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
