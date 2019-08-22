add_library(sens_loc
    )
add_library(sens_loc::sens_loc ALIAS sens_loc)
target_include_directories(sens_loc
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/sens_loc/include"
    )
target_link_libraries(sens_loc
    PRIVATE
    rang
    ${OpenCV_LIBS}
    )
