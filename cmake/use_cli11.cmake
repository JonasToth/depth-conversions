set(CLI11_INSTALL OFF CACHE BOOL "CLI11 install overwritten")
set(CLI11_SINGLE_FILE OFF CACHE BOOL "CLI11 single file overwritten")
set(BUILD_TESTING OFF CACHE BOOL "Overwrite testing from cli11")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/cli11")
mark_as_advanced(FORCE CLI11_INSTALL CLI11_SINGLE_FILE
                       CLI11_CLANG_TIDY_OPTIONS CLI11_SANITIZERS 
                       CLI11_WARNINGS_AS_ERRORS BUILD_TESTING)
message(STATUS "Value of CLI11_HAS_FILESYSTEM")
