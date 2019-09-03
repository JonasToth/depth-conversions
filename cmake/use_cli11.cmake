set(CLI11_INSTALL OFF CACHE BOOL "CLI11 install overwritten")
set(CLI11_SINGLE_FILE OFF CACHE BOOL "CLI11 single file overwritten")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/cli11")
mark_as_advanced(FORCE CLI11_INSTALL CLI11_SINGLE_FILE)
