add_executable(test_bm conversion/test_conversion.cpp)
target_link_libraries(test_bm PRIVATE nonius pthread)
