function(common_target_properties target_name)
    set_target_properties(${target_name}
        PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
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
        PRIVATE "-Wall" "-Wextra" "-Werror" "-mavx")

    if (CMAKE_CXX_COMPILER_ID EQUAL "GNU")
        target_compile_options(${target_name}
            PRIVATE "-Wno-deprecated-copy"
            )
    endif (CMAKE_CXX_COMPILER_ID EQUAL "GNU")

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

function(common_executable_options target_name)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")
    if (USE_LIBCXX)
        if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            message(STATUS "Choosen Compiler: ${CMAKE_CXX_COMPILER_ID}")
            message(FATAL_ERROR "libc++ can only be used with clang")
        endif ()
        target_compile_options(${target_name} PUBLIC "-stdlib=libc++")
        target_link_options(${target_name} PUBLIC "-lc++abi" "-stdlib=libc++")
    endif (USE_LIBCXX)
endfunction()
