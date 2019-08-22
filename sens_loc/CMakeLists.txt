add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/apps")

if (WITH_TESTING)
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/unittest")
endif (WITH_TESTING)

add_library(sens_loc
    "${CMAKE_CURRENT_LIST_DIR}/include/sens_loc/util/console.h"
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
