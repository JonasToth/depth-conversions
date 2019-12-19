FROM ubuntu:rolling
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apt-get update \
 && apt-get dist-upgrade -y \
 && apt-get install -y \
    build-essential \
    g++-9 gcc-9 \
    git \
    ninja-build \
    wget \
    unzip \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libpng-dev \
    libjpeg-dev \
 && pip3 install --upgrade pip \
 && pip install cmake

ARG cv_version="4.1.2"
RUN mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/$cv_version.zip \
 && unzip $cv_version.zip && mv opencv-$cv_version opencv && rm $cv_version.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/$cv_version.zip \
 && unzip $cv_version.zip && mv opencv_contrib-$cv_version opencv_contrib && rm $cv_version.zip \
 && cd /opencv/build_cv \
 && CC=gcc-9 CXX=g++-9 cmake /opencv/opencv \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DOPENCV_EXTRA_MODULES_PATH="/opencv/opencv_contrib/modules/" \
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
    -DOPENCV_ENABLE_MEMALIGN=OFF \
    -DOPENCV_ENABLE_NONFREE=ON \
 && ninja \
 && ninja install \
 && cd /opencv && rm -rf build_cv

ARG eigen_version="3.3.7"
RUN mkdir -p /eigen3 && cd /eigen3 \
 && wget --quiet http://bitbucket.org/eigen/eigen/get/$eigen_version.tar.gz \
 && tar xf $eigen_version.tar.gz && mv eigen* eigen && cd eigen \
 && mkdir build && cd build \
 && cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr \
 && ninja install

ARG boost_version="1.72.0"
ARG boost_version_string="1_72_0"
RUN mkdir -p /boost cd /boost \
 && wget --quiet https://dl.bintray.com/boostorg/release/${boost_version}/source/boost_${boost_version_string}.tar.gz \
 && tar xf boost_$boost_version_string.tar.gz && mv boost* boost && cd boost \
 && ./bootstrap.sh --prefix=/usr \
 && ./b2 install link=static variant=release
