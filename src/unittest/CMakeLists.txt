include(unittest_macros)

create_test(camera_models)
test_add_file(camera_models camera_models/test_pinhole.cpp)

create_test(conversion_bearing)
test_add_file(conversion_bearing conversion/test_depth_to_bearing.cpp)
test_add_file(conversion_bearing conversion/test_pathological_cases.cpp)
configure_file(conversion/data0-depth.png conversion/data0-depth.png COPYONLY)
configure_file(conversion/bearing-antidiagonal.png conversion/bearing-antidiagonal.png COPYONLY)
configure_file(conversion/bearing-diagonal.png conversion/bearing-diagonal.png COPYONLY)
configure_file(conversion/bearing-horizontal-double.png conversion/bearing-horizontal-double.png COPYONLY)
configure_file(conversion/bearing-horizontal-float.png conversion/bearing-horizontal-float.png COPYONLY)
configure_file(conversion/bearing-vertical.png conversion/bearing-vertical.png COPYONLY)
configure_file(conversion/horizontal-diff.png conversion/horizontal-diff.png COPYONLY)
configure_file(conversion/crashing-file.png conversion/crashing-file.png COPYONLY)

create_test(conversion_flexion)
configure_file(conversion/flexion-reference.png conversion/flexion-reference.png COPYONLY)

create_test(conversion_laser)
test_add_file(conversion_laser conversion/test_depth_to_laserscan.cpp)
configure_file(conversion/data0-depth-laser.png conversion/data0-depth-laser.png COPYONLY)

create_test(conversion_max_curve)
test_add_file(conversion_max_curve conversion/test_depth_to_max_curve.cpp)
configure_file(conversion/max-curve-float.png conversion/max-curve-float.png COPYONLY)
configure_file(conversion/max-curve-double.png conversion/max-curve-double.png COPYONLY)

create_test(conversion)
test_add_file(conversion conversion/test_util.cpp)

create_test(conversion_curvature)

create_test(io)
test_add_file(io io/test_image.cpp)
test_add_file(io io/test_intrinsics.cpp)

configure_file(io/example-image.jpg io/example-image.jpg COPYONLY)
configure_file(io/example-image.png io/example-image.png COPYONLY)
configure_file(io/not_an_image.txt io/not_an_image.txt COPYONLY)

create_test(math)
test_add_file(math math/test_angle_conversion.cpp)
test_add_file(math math/test_curvature.cpp)
test_add_file(math math/test_derivatives.cpp)
test_add_file(math math/test_triangles.cpp)

create_test(util)
test_add_file(util util/test_console.cpp)

create_test(version)
