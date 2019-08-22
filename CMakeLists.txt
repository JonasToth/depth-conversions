# Install the latest cmake with 'pip' if necessary
# See https://pypi.org/project/cmake/
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(master-thesis
        LANGUAGES CXX
        VERSION 0.1.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(WITH_DEP_TESTING OFF "Enable tests in thridparty modules")

# Thirdparty dependencies
include(fetch_opencv)
include(wrap_rang)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/cli11")

include(check_dependencies_built)
if (${DEPENDENCIES_BUILT})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/image-pp")
endif (${DEPENDENCIES_BUILT})
