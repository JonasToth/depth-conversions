set(GSL_CXX_STANDARD "17" CACHE STRING "GSL C++ Standard")
set(GSL_TEST OFF CACHE BOOL "GSL Testing")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/gsl")
mark_as_advanced(FORCE GSL_CXX_STANDARD
                       GSL_TEST)

add_definitions("-DGSL_THROW_ON_CONTRACT_VIOLATION=1")
