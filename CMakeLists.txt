# Install the latest cmake with 'pip' if necessary
# See https://pypi.org/project/cmake/
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Assuming the canonical version is listed in a single line
# This would be in several parts if picking up from MAJOR, MINOR, etc.
set(MAJOR_VERSION_REGEX "#define SENS_LOC_MAJOR[ \t]+(.+)")
set(MINOR_VERSION_REGEX "#define SENS_LOC_MINOR[ \t]+(.+)")
set(PATCH_VERSION_REGEX "#define SENS_LOC_PATCH[ \t]+(.+)")

# Read in the line containing the version
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/sens_loc/include/sens_loc/version.h"
             MAJOR_VERSION_STRING REGEX ${MAJOR_VERSION_REGEX})
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/sens_loc/include/sens_loc/version.h"
             MINOR_VERSION_STRING REGEX ${MINOR_VERSION_REGEX})
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/sens_loc/include/sens_loc/version.h"
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

option(WITH_TESTING ON "Enable unittests for this project")
option(WITH_DEP_TESTING OFF "Enable tests in thridparty modules")

if (WITH_TESTING)
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

include(check_dependencies_built)
if (${DEPENDENCIES_BUILT})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/sens_loc")
endif (${DEPENDENCIES_BUILT})
