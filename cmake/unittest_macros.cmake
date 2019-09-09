macro(create_test name)
    add_executable(test_${name} ${CMAKE_CURRENT_LIST_DIR}/test_${name}.cpp)
    target_link_libraries(test_${name}
                          PRIVATE sens_loc::sens_loc doctest::doctest)
    common_target_properties(test_${name})

    add_test(test_${name} test_${name})
endmacro()

macro(test_add_file test_name file_name)
    target_sources(test_${test_name}
                   PRIVATE ${CMAKE_CURRENT_LIST_DIR}/${file_name})
endmacro()
