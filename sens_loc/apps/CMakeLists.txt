add_executable(depth2bearing
    "${CMAKE_CURRENT_LIST_DIR}/depth2bearing/main_single.cpp")
add_dependencies(depth2bearing opencv)
target_link_libraries(depth2bearing
    PRIVATE
    CLI11::CLI11
    Cpp-Taskflow
    sens_loc::sens_loc
    )
set_target_properties(depth2bearing
    PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON)

add_executable(depth2bearing-batch
    "${CMAKE_CURRENT_LIST_DIR}/depth2bearing/main_batch.cpp")
add_dependencies(depth2bearing-batch opencv)
target_link_libraries(depth2bearing-batch
    PRIVATE
    CLI11::CLI11
    Cpp-Taskflow
    sens_loc::sens_loc
    )
set_target_properties(depth2bearing-batch
    PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON)

configure_file(kinect_intrinsic.txt kinect_intrinsic.txt COPYONLY)

include(GNUInstallDirs)
install(TARGETS depth2bearing
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS depth2bearing-batch
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
