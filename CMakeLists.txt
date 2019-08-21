# Install the latest cmake with 'pip' if necessary
# See https://pypi.org/project/cmake/
cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(master-thesis
        LANGUAGES CXX
        VERSION 0.1.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
