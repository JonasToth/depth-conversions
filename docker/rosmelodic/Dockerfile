FROM ros:melodic
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apt-get update \
 && apt-get install -y \
    libeigen3-dev \
    git \
    python3-pip \
    ninja-build \
    software-properties-common \
 && add-apt-repository ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y  \
    gcc-9 g++-9 \
    unzip wget \
 && pip3 install --upgrade pip \
 && pip install cmake

ARG cv_version="4.2.0"
RUN mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/$cv_version.zip \
 && unzip $cv_version.zip && rm $cv_version.zip \
 && cd /opencv/build_cv \
 && CC=gcc CXX=g++ cmake /opencv/opencv-$cv_version/ \
    -G Ninja \
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
    -DOPENCV_ENABLE_NONFREE=ON \
&& ninja \
&& ninja install \
&& cd / && rm -rf /opencv

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
 && tar xf boost_$boost_version_string.tar.gz && cd boost_$boost_version_string \
 && ./bootstrap.sh --prefix=/usr \
 && ./b2 install link=static variant=release
