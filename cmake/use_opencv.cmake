include(opencv_options)
include(ExternalProject)
ExternalProject_Add(opencv
  DOWNLOAD_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/opencv"
  CMAKE_ARGS "${opencv_options};-DCMAKE_INSTALL_PREFIX=/usr"

  # Build step
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/opencv-build"

  # Install step
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/opencv-install"
  INSTALL_COMMAND sh -c "DESTDIR=${CMAKE_BINARY_DIR}/opencv-install ${CMAKE_MAKE_PROGRAM} install"

  # Test step
  TEST_BEFORE_INSTALL 0
  TEST_AFTER_INSTALL 0
  # TEST_COMMAND "${CMAKE_MAKE_PROGRAM} test"

  # Output logging
  LOG_DOWNLOAD 0
  LOG_UPDATE 0
  LOG_CONFIGURE 1
  LOG_BUILD 1
  LOG_TEST 1
  LOG_INSTALL 1
)
set_target_properties(opencv PROPERTIES EXCLUDE_FROM_ALL TRUE)

if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/opencv-install/usr")
	set(opencv_built TRUE)
    find_package(OpenCV REQUIRED
                 PATHS "${CMAKE_CURRENT_BINARY_DIR}/opencv-install/usr")

    # Hide this variable in cmake-gui.
    mark_as_advanced(FORCE OpenCV_DIR)
else ()
	set(opencv_built FALSE)
endif (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/opencv-install/usr")