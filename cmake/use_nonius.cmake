add_library(nonius::nonius INTERFACE)
target_include_directories(rang 
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nonius/include/")
