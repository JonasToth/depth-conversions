set(TF_BUILD_BENCHMARKS OFF CACHE BOOL "Taskflow benchmarks overwritten")
set(TF_BUILD_EXAMPLES OFF CACHE BOOL "Taskflow examples overwritten")
set(TF_BUILD_TESTS OFF CACHE BOOL "Taskflow tests overwritten")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/cpp-taskflow")
mark_as_advanced(FORCE TF_BUILD_BENCHMARKS 
                       TF_BUILD_EXAMPLES
                       TF_BUILD_TESTS)
