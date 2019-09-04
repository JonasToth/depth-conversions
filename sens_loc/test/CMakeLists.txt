configure_file(depth2bearing/kinect_intrinsic.txt
               depth2bearing/kinect_intrinsic.txt COPYONLY)
configure_file(depth2bearing/data0-depth.png
               depth2bearing/data0-depth.png COPYONLY)
configure_file(depth2bearing/data1-depth.png
               depth2bearing/data1-depth.png COPYONLY)
configure_file(depth2bearing/data002-depth.png
               depth2bearing/data002-depth.png COPYONLY)
configure_file(depth2bearing/data003-depth.png
               depth2bearing/data003-depth.png COPYONLY)
configure_file(depth2bearing/data004-depth.png
               depth2bearing/data004-depth.png COPYONLY)

configure_file(depth2bearing/test_depth2bearing.sh
               depth2bearing/test_depth2bearing.sh COPYONLY)
add_test(NAME test_depth2bearing_app
    COMMAND
    sh -c "./test_depth2bearing.sh \
           $<TARGET_FILE:depth2bearing> \
           ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
    WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/depth2bearing"
    )

configure_file(depth2bearing/test_depth2bearing-batch.sh
               depth2bearing/test_depth2bearing-batch.sh COPYONLY)
add_test(NAME test_depth2bearing_batch
    COMMAND
    sh -c "./test_depth2bearing-batch.sh \
           $<TARGET_FILE:depth2bearing-batch> \
           ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
    WORKING_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/depth2bearing"
    )
