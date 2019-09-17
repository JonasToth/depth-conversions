FROM ubuntu:18.04

RUN apt-get update \
 && apt-get dist-upgrade -y \
 && apt-get install -y \
    build-essential \
    git \
    ninja-build \
    software-properties-common \
    wget \
    unzip \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libpng-dev \
    libjpeg-dev \
 && add-apt-repository ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y  \
    gcc-9 g++-9 \
 && pip3 install --upgrade pip \
 && pip install cmake \
 && mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/4.1.1.zip \
 && unzip 4.1.1.zip && rm 4.1.1.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/4.1.1.zip \
 && unzip 4.1.1.zip && rm 4.1.1.zip \
 && cd /opencv/build_cv \
 && CC=gcc-9 CXX=g++-9 cmake /opencv/opencv-4.1.1/ \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DOPENCV_EXTRA_MODULES_PATH="/opencv/opencv_contrib-4.1.1/modules/" \
    -DBUILD_opencv_java_bindings_generator=OFF \
    -DBUILD_opencv_js=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_SHARED_LIBS=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DBUILD_TESTS=OFF \
    -DINSTALL_TESTS=OFF \
    -DENABLE_LTO=OFF \
    -DOPENCV_PYTHON3_VERSION=OFF \
&& ninja \
&& ninja install \
&& cd / && rm -rf /opencv
