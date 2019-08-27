add_library(sens_loc
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/console.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/image.h"
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/io/intrinsics.h"
    "${CMAKE_CURRENT_LIST_DIR}/lib/io/intrinsics.cpp"
    )
add_library(sens_loc::sens_loc ALIAS sens_loc)
target_include_directories(sens_loc
    PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
    )
target_link_libraries(sens_loc
    PUBLIC
    rang
    ${OpenCV_LIBS}
    )
set_target_properties(sens_loc
    PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON)
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
