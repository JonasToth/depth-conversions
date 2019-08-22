macro(create_test name)
    add_executable(test_${name} ${CMAKE_CURRENT_LIST_DIR}/test_${name}.cpp)
    target_link_libraries(test_${name}
                          PRIVATE sens_loc::sens_loc doctest::doctest)
    add_test(test_${name} test_${name})
    set_target_properties(test_${name}
        PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON)
endmacro()

macro(test_add_file test_name file_name)
    target_sources(test_${test_name}
                   PUBLIC ${CMAKE_CURRENT_LIST_DIR}/${file_name})
endmacro()
