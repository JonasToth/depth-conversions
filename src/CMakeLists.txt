find_package(Threads REQUIRED)

add_library(sens_loc INTERFACE)
target_sources(sens_loc INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/version.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/camera_models/pinhole.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/camera_models/equirectangular.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/camera_models/utility.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_bearing.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_curvature.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_flexion.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_laserscan.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_to_max_curve.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/depth_scaling.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/conversion/util.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/image.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/intrinsics.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/angle_conversion.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/constants.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/coordinate.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/curvature.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/derivatives.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/eigen_types.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/image.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/scaling.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/math/triangles.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/preprocess/filter.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/console.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/correctness_util.h"
    )
add_library(sens_loc::sens_loc ALIAS sens_loc)
target_include_directories(sens_loc
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
target_link_libraries(sens_loc
    INTERFACE
    GSL
    rang
    Threads::Threads
    taskflow::taskflow
    Eigen3::Eigen
    ${OpenCV_LIBS}
    )
target_compile_features(sens_loc INTERFACE cxx_std_17)
add_dependencies(sens_loc dependencies)

install(TARGETS sens_loc
        EXPORT sens_loc
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        )

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/apps")
if (WITH_TESTING)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/unittest")
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/test")
endif (WITH_TESTING)

if (WITH_BENCHMARK)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/benchmarks")
endif (WITH_BENCHMARK)
