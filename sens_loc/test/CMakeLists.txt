configure_file(depth2bearing/test_depth2bearing.sh
               depth2bearing/test_depth2bearing.sh COPYONLY)
configure_file(depth2bearing/kinect_intrinsic.txt
               depth2bearing/kinect_intrinsic.txt COPYONLY)
configure_file(depth2bearing/data0-depth.png
               depth2bearing/data0-depth.png COPYONLY)
add_test(NAME test_depth2bearing_app
    COMMAND
    sh -c "./test_depth2bearing.sh \
           $<TARGET_FILE:depth2bearing> \
           ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
    WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/depth2bearing"
    )
