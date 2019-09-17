function(common_target_properties target_name)
    set_target_properties(${target_name}
        PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
        )
    if (WITH_CONTRACT_EXCEPTION)
        target_compile_definitions(${target_name}
            PUBLIC "GSL_THROW_ON_CONTRACT_VIOLATION=1")
    else ()
        target_compile_definitions(${target_name}
            PUBLIC "GSL_TERMINATE_ON_CONTRACT_VIOLATION")
    endif (WITH_CONTRACT_EXCEPTION)
    target_compile_options(${target_name}
        PRIVATE
        "-Wall" "-Wextra" "-mavx")

    if (CXX_COMPILER_ID EQUAL "GNU")
        target_compile_options(${target_name}
            PRIVATE
            "-Wno-deprecated-copy"
            )
    endif ()

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
endfunction()
