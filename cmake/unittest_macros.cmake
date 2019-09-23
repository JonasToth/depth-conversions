macro(create_test name source_file)
    add_executable(test_${name} ${CMAKE_CURRENT_LIST_DIR}/${source_file})

    # NOTE: `doctest` has to come before `sens_loc`, because `cpp-taskflow`
    # has `doctest` in the repository as well, and its version of it would be
    # used, which is undesired!
    target_link_libraries(test_${name}
                          PRIVATE doctest::doctest sens_loc::sens_loc)
    common_target_properties(test_${name})
    common_executable_options(test_${name})

    add_test(test_${name} test_${name})
endmacro()

macro(test_add_file test_name file_name)
    target_sources(test_${test_name}
                   PRIVATE ${CMAKE_CURRENT_LIST_DIR}/${file_name})
endmacro()
