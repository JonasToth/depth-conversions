function(sanitizer_config target_name)
    if (WITH_UBSAN)
        target_compile_options(${target_name} PUBLIC "-fsanitize=undefined")
        target_link_options(${target_name} PUBLIC "-fsanitize=undefined")
    endif (WITH_UBSAN)

    if (WITH_ASAN)
        target_compile_options(${target_name} PUBLIC "-fsanitize=address")
        target_link_options(${target_name} PUBLIC "-fsanitize=address")
    endif (WITH_ASAN)

    if (WITH_TSAN)
        target_compile_options(${target_name} PUBLIC "-fsanitize=thread")
        target_link_options(${target_name} PUBLIC "-fsanitize=thread")
    endif (WITH_TSAN)

    if (WITH_MSAN)
        target_compile_options(${target_name}
            PUBLIC
            "-fsanitize=memory"
            "-fsanitize-blacklist=${PROJECT_SOURCE_DIR}/scripts/msan_suppressions"
            "-fsanitize-memory-track-origins"
            "-fno-omit-frame-pointer")
        target_link_options(${target_name}
            PUBLIC
            "-fsanitize=memory"
            "-fsanitize-memory-track-origins"
            "-fno-omit-frame-pointer")
    endif (WITH_MSAN)
endfunction(sanitizer_config)

function(common_target_properties target_name)
    add_dependencies(${target_name} dependencies)
    set_target_properties(${target_name}
        PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ${WITH_PIC}
        )
    if (WITH_TEST_COVERAGE)
        target_compile_options(${target_name} 
            PRIVATE "--coverage" "-fprofile-arcs" "-ftest-coverage")
        target_link_options(${target_name} PRIVATE "--coverage")
    endif (WITH_TEST_COVERAGE)

    if (WITH_CONTRACT_EXCEPTION)
        target_compile_definitions(${target_name}
            PUBLIC "GSL_THROW_ON_CONTRACT_VIOLATION=1")
    else ()
        target_compile_definitions(${target_name}
            PUBLIC "GSL_TERMINATE_ON_CONTRACT_VIOLATION")
    endif (WITH_CONTRACT_EXCEPTION)
    target_compile_options(${target_name}
        PRIVATE
            "-Wall"
            "-Wextra"
            "$<$<BOOL:{WITH_FAST_MATH}>:-ffast-math>"
            "$<$<BOOL:${WITH_WERROR}>:-Werror>"
            "$<$<BOOL:${WITH_MARCH_NATIVE}>:-march=native>"
            "$<$<BOOL:${WITH_SSE42}>:-msse4.2>"
            "$<$<BOOL:${WITH_AVX}>:-mavx>"
            "$<$<BOOL:${WITH_AVX2}>:-mavx2>"
            )
    sanitizer_config(${target_name})

    if (WITH_IPO)
        set_target_properties(${target_name}
                              PROPERTIES INTERPROCEDURAL_OPTIMIZATION ON)
    else ()
        set_target_properties(${target_name}
                              PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
    endif (WITH_IPO)
endfunction(common_target_properties)

# This function creates an executable with name \c name and links it to
# common libraries from this project.
function(add_tool name main_file)
    add_executable("${name}" "${main_file}")
    target_include_directories("${name}" PRIVATE "${CMAKE_CURRENT_LIST_DIR}")

    target_link_libraries("${name}"
        PRIVATE
        fmt::fmt
        CLI11::CLI11
        sens_loc::sens_loc
        sens_loc::batch_processing
        )
    common_target_properties("${name}")
    install(TARGETS "${name}" RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endfunction (add_tool)
