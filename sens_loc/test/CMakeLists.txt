configure_file(depth2x/kinect_intrinsic.txt
               depth2x/kinect_intrinsic.txt COPYONLY)
configure_file(depth2x/data0-depth.png
               depth2x/data0-depth.png COPYONLY)
configure_file(depth2x/data1-depth.png
               depth2x/data1-depth.png COPYONLY)
configure_file(depth2x/data002-depth.png
               depth2x/data002-depth.png COPYONLY)
configure_file(depth2x/data003-depth.png
               depth2x/data003-depth.png COPYONLY)
configure_file(depth2x/data004-depth.png
               depth2x/data004-depth.png COPYONLY)

configure_file(depth2x/test_depth2x.sh
               depth2x/test_depth2x.sh COPYONLY)
add_test(NAME test_depth2x_app
    COMMAND
    sh -c "./test_depth2x.sh \
           $<TARGET_FILE:depth2x> \
           ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
    WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/depth2x"
    )

configure_file(depth2x/test_depth2x-batch.sh
               depth2x/test_depth2x-batch.sh COPYONLY)
add_test(NAME test_depth2x_batch
    COMMAND
    sh -c "./test_depth2x-batch.sh \
           $<TARGET_FILE:depth2x-batch> \
           ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
    WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/depth2x"
    )
