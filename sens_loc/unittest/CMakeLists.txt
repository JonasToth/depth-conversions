add_executable(test_util
    ${CMAKE_CURRENT_LIST_FILE}/test_util.cpp
    ${CMAKE_CURRENT_LIST_FILE}/util/test_console.cpp)
target_link_libraries(test_console PRIVATE sens_loc doctest::doctest)
add_test(test_util test_util)
