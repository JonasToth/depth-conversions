add_library(nonius INTERFACE)
target_include_directories(nonius SYSTEM
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nonius/include/")
