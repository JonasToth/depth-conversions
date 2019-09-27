macro(create_bm name source_file)
    add_executable(bm_${name} ${source_file})
    target_link_libraries(bm_${name}
                          PRIVATE nonius pthread sens_loc::sens_loc)
    common_target_properties(bm_${name})
    if (WITH_BENCHMARK_JUNIT_REPORT)
        add_test(NAME bm_${name}
                 COMMAND bm_${name} -r junit -o bm_${name}.xml)
    else ()
        add_test(NAME bm_${name}
                 COMMAND bm_${name})
    endif ()
endmacro()
