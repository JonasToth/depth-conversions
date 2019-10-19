FROM ubuntu:18.04
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apt-get update \
 && apt-get install -y wget gnupg2 \
 && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
 && echo "deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic-8 main" >> /etc/apt/sources.list \
 && echo "deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic-9 main" >> /etc/apt/sources.list \
 && apt-get update \
 && apt-get dist-upgrade -y \
 && apt-get install -y \
    build-essential \
    git \
    ninja-build \
    software-properties-common \
    unzip \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libpng-dev \
    libjpeg-dev \
    rsync \
    unzip \
 && add-apt-repository ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y  \
    clang-8 lld-8  \
    clang-9 clang-tools-9 lld-9 clang-tidy-9 \
    libc++1-9 libc++-9-dev libc++abi1-9 libc++abi-9-dev \
    gcc-7 g++-7 \
    gcc-8 g++-8 \
    gcc-9 g++-9 \
    lcov gcovr \
 && pip3 install --upgrade pip \
 && pip install cmake

ARG cv_version="4.1.1"
RUN mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && cd /opencv/build_cv \
 && CC=gcc-9 CXX=g++-9 cmake /opencv/opencv-$cv_version/ \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DOPENCV_EXTRA_MODULES_PATH="/opencv/opencv_contrib-$cv_version/modules/" \
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
&& cd /opencv && rm -rf build_cv
