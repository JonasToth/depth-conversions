set(DOCTEST_NO_INSTALL ON CACHE BOOL "DOCTEST no install overwritten")
set(DOCTEST_WITH_MAIN_IN_STATIC_LIB ON 
    CACHE BOOL "DOCTEST static lib overwritten")
set(DOCTEST_WITH_TESTS OFF CACHE BOOL "DOCTEST testing overwritten")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/doctest")
mark_as_advanced(FORCE DOCTEST_NO_INSTALL 
                       DOCTEST_WITH_MAIN_IN_STATIC_LIB 
                       DOCTEST_WITH_TESTS)
