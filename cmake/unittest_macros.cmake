macro(create_test name source_file)
    add_executable(test_${name} ${CMAKE_CURRENT_LIST_DIR}/${source_file})

    # NOTE: `doctest` has to come before `sens_loc`, because `cpp-taskflow`
    # has `doctest` in the repository as well, and its version of it would be
    # used, which is undesired!
    target_link_libraries(test_${name}
                          PRIVATE doctest::doctest sens_loc::sens_loc)
    common_target_properties(test_${name})

    add_test(test_${name} test_${name})
endmacro()

macro(test_add_file test_name file_name)
    target_sources(test_${test_name}
                   PRIVATE ${CMAKE_CURRENT_LIST_DIR}/${file_name})
endmacro()

macro (compile_failure name source_file test_macro)
    add_executable(${name}_${test_macro} ${source_file})
    target_link_libraries(${name}_${test_macro}
                          PRIVATE sens_loc::sens_loc)
    set_target_properties(${name}_${test_macro}
                          PROPERTIES EXCLUDE_FROM_ALL TRUE
                          EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    target_compile_definitions(${name}_${test_macro} PRIVATE ${test_macro})
    add_test(NAME fail_${name}_${test_macro}
             COMMAND ${CMAKE_COMMAND} --build .
                                      --target ${name}_${test_macro}
                                      --config $<CONFIGURATION>
             WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    set_tests_properties(fail_${name}_${test_macro}
                         PROPERTIES WILL_FAIL TRUE)
endmacro ()
