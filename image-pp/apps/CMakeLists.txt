add_executable(depth2bearing "")
add_dependencies(depth2bearing opencv)
target_sources(depth2bearing 
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/depth2bearing/main.cpp")
target_link_libraries(depth2bearing
    PRIVATE
    ${OpenCV_LIBS}
    CLI11::CLI11
    )
set_target_properties(depth2bearing
    PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
install(TARGETS depth2bearing
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
