set(CustomOpenCV "" CACHE PATH "Custom directory for opencv install in the system")
find_package(OpenCV HINTS ${CustomOpenCV})

if (NOT OpenCV_FOUND)
    message(STATUS "Could not find OpenCV in the system. Building custom")
    include(opencv_options)
    include(ExternalProject)

    ExternalProject_Add(opencv
      PREFIX "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv"
      DOWNLOAD_COMMAND sh -c "wget --quiet \
         https://github.com/opencv/opencv/archive/4.1.1.zip \
      && unzip -o 4.1.1.zip \
      && rsync -a opencv-4.1.1/ ./ \
      && rm -r opencv-4.1.1 4.1.1.zip \
      && wget --quiet \
         https://github.com/opencv/opencv_contrib/archive/4.1.1.zip \
      && unzip -o 4.1.1.zip \
      && mv opencv_contrib-4.1.1 opencv_contrib \
      && rm 4.1.1.zip"

      SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv/src"
      CMAKE_ARGS "${opencv_options};-DCMAKE_INSTALL_PREFIX=/usr"

      # Build step
      BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv-build"

      # Install step
      INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv-install"
      INSTALL_COMMAND sh -c "DESTDIR=${CMAKE_BINARY_DIR}/third_party/opencv-install ${CMAKE_MAKE_PROGRAM} install"

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

    if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv-install/usr")
        set(opencv_built TRUE)
        find_package(OpenCV REQUIRED
                     PATHS "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv-install/usr")

        # Hide this variable in cmake-gui.
        mark_as_advanced(FORCE OpenCV_DIR)
    else ()
        set(opencv_built FALSE)
    endif (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/third_party/opencv-install/usr")
# OpenCV was found from the system
else ()
    message(STATUS "Found existing opencv. Assuming it includes non-free features!")
    add_custom_target(opencv)
    set(opencv_build TRUE)
endif (NOT OpenCV_FOUND)
