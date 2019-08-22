# Install the latest cmake with 'pip' if necessary
# See https://pypi.org/project/cmake/
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(master-thesis
        LANGUAGES CXX
        VERSION 0.1.0)
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

include(check_dependencies_built)
if (${DEPENDENCIES_BUILT})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/sens_loc")
endif (${DEPENDENCIES_BUILT})
