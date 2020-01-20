set(FMT_DOC OFF CACHE BOOL "FMT Doc overwritten")
set(FMT_FUZZ OFF CACHE BOOL "FMT Fuzz overwritten")
set(FMT_INSTALL OFF CACHE BOOL "FMT Install overwritten")
set(FMT_PEDANTIC ON CACHE BOOL "FMT Pedantic overwritten")
set(FMT_TEST OFF CACHE BOOL "FMT Test overwritten")
set(FMT_WERROR OFF CACHE BOOL "FMT WError overwritten")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt")
mark_as_advanced(FORCE FMT_DOC FMT_FUZZ FMT_INSTALL FMT_PEDANTIC FMT_TEST
                       FMT_WERROR)

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
