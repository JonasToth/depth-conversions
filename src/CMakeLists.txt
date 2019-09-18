find_package(Threads REQUIRED)
find_package (Eigen3 3.3 NO_MODULE)

add_library(sens_loc
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/version.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/camera_models/pinhole.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_bearing.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_curvature.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_flexion.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_laserscan.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_max_curve.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/util.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/image.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/intrinsics.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/angle_conversion.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/constants.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/curvature.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/derivatives.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/console.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/correctness_util.h"
    "${CMAKE_CURRENT_LIST_DIR}/lib/io/intrinsics.cpp"
    )
add_library(sens_loc::sens_loc ALIAS sens_loc)
target_include_directories(sens_loc
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
    )
target_link_libraries(sens_loc
    PUBLIC
    GSL
    rang
    Threads::Threads
    taskflow::taskflow
    ${OpenCV_LIBS}
    )
if (TARGET Eigen3::Eigen)
    target_link_libraries(sens_loc PRIVATE Eigen3::Eigen)
endif (TARGET Eigen3::Eigen)

common_target_properties(sens_loc)

install(TARGETS sens_loc
        EXPORT sens_loc
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/apps")
if (WITH_TESTING)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/unittest")
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/test")
endif (WITH_TESTING)

if (WITH_BENCHMARK)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/benchmarks")
endif (WITH_BENCHMARK)
