configure_file(depth2x/kinect_intrinsic.txt
               depth2x/kinect_intrinsic.txt COPYONLY)
configure_file(depth2x/laser_intrinsic.txt
               depth2x/laser_intrinsic.txt COPYONLY)

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
configure_file(depth2x/data005-depth.png
               depth2x/data005-depth.png COPYONLY)
configure_file(depth2x/data006-depth.png
               depth2x/data006-depth.png COPYONLY)

configure_file(depth2x/laserscan-0-depth.png
               depth2x/laserscan-0-depth.png COPYONLY)
configure_file(depth2x/laserscan-1-depth.png
               depth2x/laserscan-1-depth.png COPYONLY)


macro (add_app_test test_name)
    configure_file(depth2x/${test_name}.sh
                   depth2x/${test_name}.sh COPYONLY)

    add_test(NAME ${test_name}
        COMMAND
        sh -c "./${test_name}.sh \
               $<TARGET_FILE:depth2x> \
               ${PROJECT_SOURCE_DIR}/scripts/log_helpers.sh"
        WORKING_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}/depth2x"
        )
endmacro(add_app_test)

add_app_test(test_depth2x)
add_app_test(test_depth2x_bearing)
add_app_test(test_depth2x_flexion)
add_app_test(test_depth2x_gaussian_curvature)
add_app_test(test_depth2x_mean_curvature)
add_app_test(test_depth2x_max_curve)
add_app_test(test_depth2x_range)
add_app_test(test_depth2x_scale)
