macro(create_bm name source_file)
    add_executable(bm_${name} ${source_file})
    target_link_libraries(bm_${name}
                          PRIVATE nonius pthread sens_loc::sens_loc)
    common_target_properties(bm_${name})
    common_executable_options(bm_${name})
    add_test(bm_${name} bm_${name})
endmacro()
