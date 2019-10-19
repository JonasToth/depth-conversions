FROM ubuntu:12.04
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apt-get update \
 && apt-get install -y wget python-software-properties rsync unzip \
 && wget --quiet https://github.com/Kitware/CMake/releases/download/v3.15.3/cmake-3.15.3-Linux-x86_64.sh \
 && sh cmake-3.15.3-Linux-x86_64.sh --prefix=/usr/local --skip-license \
 && rm cmake-3.* \
 && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y g++-9 gcc-9 ninja-build

ARG cv_version="4.1.2"
RUN mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && cd /opencv/build_cv \
 && CC=gcc-9 CXX=g++-9 cmake /opencv/opencv-$cv_version/ \
    -G Ninja \
    -DCMAKE_EXE_LINKER_FLAGS="-static-libstdc++ -static-libgcc" \
    -DCMAKE_BUILD_TYPE=Release \
    -DOPENCV_EXTRA_MODULES_PATH="/opencv/opencv_contrib-$cv_version/modules/" \
    -DBUILD_opencv_java_bindings_generator=OFF \
    -DBUILD_opencv_js=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_SHARED_LIBS=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DCPU_BASELINE:STRING="SSE3" \
    -DCPU_DISPATCH:STRING="SSE4_1;SSE4_2;AVX;FP16;AVX2" \
    -DBUILD_TESTS=OFF \
    -DINSTALL_TESTS=OFF \
    -DENABLE_LTO=OFF \
    -DOPENCV_PYTHON3_VERSION=OFF \
    -DOPENCV_ENABLE_MEMALIGN=OFF \
 && ninja \
 && ninja install \
 && cd / && rm -rf /opencv

ARG eigen_version="3.3.7"
RUN mkdir -p /eigen3 && cd /eigen3 \
 && wget --quiet http://bitbucket.org/eigen/eigen/get/$eigen_version.tar.gz \
 && tar xf $eigen_version.tar.gz && mv eigen* eigen-$eigen_version && cd eigen-$eigen_version \
 && mkdir build && cd build \
 && cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr \
 && ninja install \
 && cd / && rm -rf /eigen3
