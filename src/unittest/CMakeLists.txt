include(unittest_macros)

create_test(camera_models camera_models/test_camera_models.cpp)
test_add_file(camera_models camera_models/test_pinhole.cpp)
test_add_file(camera_models camera_models/test_equirectangular.cpp)

# Conversion tests all require this file.
configure_file(conversion/data0-depth.png conversion/data0-depth.png COPYONLY)
configure_file(conversion/laserscan-depth.png conversion/laserscan-depth.png COPYONLY)

create_test(conversion_bearing conversion/test_conversion_bearing.cpp)
test_add_file(conversion_bearing conversion/test_conversion_bearing_pathological.cpp)
configure_file(conversion/bearing-antidiagonal.png
               conversion/bearing-antidiagonal.png COPYONLY)
configure_file(conversion/bearing-diagonal.png
               conversion/bearing-diagonal.png COPYONLY)
configure_file(conversion/bearing-horizontal.png
               conversion/bearing-horizontal.png COPYONLY)
configure_file(conversion/bearing-vertical.png
               conversion/bearing-vertical.png COPYONLY)
configure_file(conversion/crashing-file.png conversion/crashing-file.png COPYONLY)
configure_file(conversion/bearing-horizontal-laserscan-reference.png
               conversion/bearing-horizontal-laserscan-reference.png COPYONLY)

create_test(conversion_curvature conversion/test_conversion_curvature.cpp)
configure_file(conversion/mean-reference.png conversion/mean-reference.png COPYONLY)
configure_file(conversion/mean-laserscan-reference.png
               conversion/mean-laserscan-reference.png COPYONLY)
configure_file(conversion/gauss-reference.png conversion/gauss-reference.png COPYONLY)
configure_file(conversion/gauss-laserscan-reference.png
               conversion/gauss-laserscan-reference.png COPYONLY)

create_test(conversion_flexion conversion/test_conversion_flexion.cpp)
configure_file(conversion/flexion-reference.png conversion/flexion-reference.png COPYONLY)
configure_file(conversion/flexion-laserscan-reference.png conversion/flexion-laserscan-reference.png COPYONLY)

create_test(conversion_laser conversion/test_conversion_laser.cpp)
configure_file(conversion/data0-depth-laser.png conversion/data0-depth-laser.png COPYONLY)

create_test(conversion_max_curve conversion/test_conversion_max_curve.cpp)
configure_file(conversion/max-curve-double.png conversion/max-curve-double.png COPYONLY)
configure_file(conversion/max-curve-laserscan.png conversion/max-curve-laserscan.png COPYONLY)

create_test(conversion_scaling conversion/test_conversion_scaling.cpp)
configure_file(conversion/scale-offset.png conversion/scale-offset.png COPYONLY)
configure_file(conversion/scale-up.png conversion/scale-up.png COPYONLY)

create_test(conversion_util conversion/test_util.cpp)

create_test(io io/test_io.cpp)
test_add_file(io io/test_image.cpp)
test_add_file(io io/test_intrinsics.cpp)
configure_file(io/example-image.jpg io/example-image.jpg COPYONLY)
configure_file(io/example-image.png io/example-image.png COPYONLY)
configure_file(io/not_an_image.txt io/not_an_image.txt COPYONLY)

create_test(math math/test_math.cpp)
test_add_file(math math/test_angle_conversion.cpp)
test_add_file(math math/test_coordinate.cpp)
test_add_file(math math/test_curvature.cpp)
test_add_file(math math/test_derivatives.cpp)
test_add_file(math math/test_image.cpp)
test_add_file(math math/test_scaling.cpp)
test_add_file(math math/test_triangles.cpp)

configure_file(conversion/data0-depth.png preprocess/data0-depth.png COPYONLY)
configure_file(conversion/laserscan-depth.png preprocess/laserscan-depth.png COPYONLY)
create_test(preprocess preprocess/test_pp.cpp)
test_add_file(preprocess preprocess/test_bilateral_filter.cpp)

create_test(util util/test_util.cpp)
test_add_file(util util/test_console.cpp)
test_add_file(util util/test_version.cpp)

create_test(util_terminate util/test_terminate.cpp)
set_tests_properties(test_util_terminate PROPERTIES WILL_FAIL TRUE)
create_test(util_terminate2 util/test_terminate2.cpp)
set_tests_properties(test_util_terminate2 PROPERTIES WILL_FAIL TRUE)

add_subdirectory(static_correctness)
