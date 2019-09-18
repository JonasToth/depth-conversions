# Install the latest cmake with 'pip' if necessary
# See https://pypi.org/project/cmake/
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Assuming the canonical version is listed in a single line
# This would be in several parts if picking up from MAJOR, MINOR, etc.
set(MAJOR_VERSION_REGEX "#define SENS_LOC_MAJOR[ \t]+(.+)")
set(MINOR_VERSION_REGEX "#define SENS_LOC_MINOR[ \t]+(.+)")
set(PATCH_VERSION_REGEX "#define SENS_LOC_PATCH[ \t]+(.+)")

# Read in the line containing the version
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/include/sens_loc/version.h"
             MAJOR_VERSION_STRING REGEX ${MAJOR_VERSION_REGEX})
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/include/sens_loc/version.h"
             MINOR_VERSION_STRING REGEX ${MINOR_VERSION_REGEX})
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/src/include/sens_loc/version.h"
             PATCH_VERSION_STRING REGEX ${PATCH_VERSION_REGEX})
# Pick out just the version
string(REGEX REPLACE ${MAJOR_VERSION_REGEX} "\\1" MAJOR_VERSION_STRING "${MAJOR_VERSION_STRING}")
string(REGEX REPLACE ${MINOR_VERSION_REGEX} "\\1" MINOR_VERSION_STRING "${MINOR_VERSION_STRING}")
string(REGEX REPLACE ${PATCH_VERSION_REGEX} "\\1" PATCH_VERSION_STRING "${PATCH_VERSION_STRING}")


project(master-thesis
        LANGUAGES CXX
        VERSION "${MAJOR_VERSION_STRING}.${MINOR_VERSION_STRING}.${PATCH_VERSION_STRING}")
message(STATUS "Version: ${MAJOR_VERSION_STRING}.${MINOR_VERSION_STRING}.${PATCH_VERSION_STRING}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(WITH_BENCHMARK ON "Enable benchmarks for this project")
option(WITH_TESTING ON "Enable unittests for this project")
option(WITH_DEP_TESTING OFF "Enable tests in thridparty modules")
option(WITH_CONTRACT_EXCEPTION ON "Enable contract throwing exceptions on contract violation")

if (WITH_TESTING)
    find_program(GCOV_PATH gcov)
    find_program(LCOV_PATH lcov)

    if(NOT GCOV_PATH OR NOT LCOV_PATH)
        message(WARNING "gcov/lcov not found! Can not collect test coverage reports...")
    else ()
        message(STATUS "Found coverage tools")
        option(WITH_TEST_COVERAGE OFF "Enable the collection of test coverage")
    endif (NOT GCOV_PATH OR NOT LCOV_PATH)
endif (WITH_TESTING)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    option(WITH_UBSAN OFF "Enable Undefined Behaviour sanitizer")
    option(WITH_ASAN OFF "Enable Undefined Behaviour sanitizer")
    option(WITH_TSAN OFF "Enable Thread Sanitizer (data races)")
endif ()

if (WITH_TESTING OR WITH_BENCHMARK)
    set(BUILD_TESTING "" ON)
    enable_testing()
endif ()

include(GNUInstallDirs)

# Thirdparty dependencies
include(use_opencv)
include(use_rang)
include(use_cli11)
include(use_doctest)
include(use_taskflow)
include(use_gsl)
include(use_fmtlib)
include(use_nonius)

# cmake utility code for the repository
include(check_dependencies_built)
include(configuration_functions)
if (${DEPENDENCIES_BUILT})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
endif (${DEPENDENCIES_BUILT})
