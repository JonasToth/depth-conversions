set(BOOST_VERSION "1.73.0")
set(BOOST_VERSION_STRING "1_73_0")
find_package(Boost ${BOOST_VERSION} EXACT QUIET)

if (NOT Boost_FOUND)
    include(ExternalProject)

    message(STATUS "Using bundled boost - Version ${BOOST_VERSION}")
    set(BOOST_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost/src")
    set(BOOST_URL
        "https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_STRING}.tar.gz")

    ExternalProject_Add(boost
      PREFIX "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost"
      STAMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost/stamp"
      URL ${BOOST_URL}

      BUILD_IN_SOURCE 1
      SOURCE_DIR ${BOOST_SOURCE}
      CONFIGURE_COMMAND ${BOOST_SOURCE}/bootstrap.sh
        --prefix=${CMAKE_CURRENT_BINARY_DIR}/third_party/boost-install
      BUILD_COMMAND ./b2 install link=static variant=release --without-python

      INSTALL_COMMAND ""
      INSTALL_DIR ""

      # Output logging
      LOG_DOWNLOAD 1
      LOG_UPDATE 0
      LOG_CONFIGURE 1
      LOG_BUILD 1
      LOG_TEST 0
      LOG_INSTALL 0
    )
    set_target_properties(boost PROPERTIES EXCLUDE_FROM_ALL TRUE)

    if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost-install/lib")
        set(boost_built TRUE)
        find_package(Boost ${BOOST_VERSION} REQUIRED
                     PATHS "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost-install/lib/cmake"
                     NO_DEFAULT_PATH
                     NO_SYSTEM_ENVIRONMENT_PATH
                     )

        # Hide this variable in cmake-gui.
        mark_as_advanced(FORCE Boost_DIR)
    else ()
        set(boost_built FALSE)
    endif (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/third_party/boost-install/lib")
else ()
    message(STATUS "Found required Boost!")
    add_custom_target(boost)
    set(boost_built TRUE)
endif (NOT Boost_FOUND)
