FROM ros:kinetic-perception
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apt-get update \
 && apt-get install -y \
    git \
    python3-pip \
    ninja-build \
    software-properties-common \
    wget \
 && add-apt-repository ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y  \
    gcc-9 g++-9 \
 && pip3 install --upgrade pip \
 && pip install cmake

ARG eigen_version="3.3.7"
RUN mkdir -p /eigen3 && cd /eigen3 \
 && wget --quiet http://bitbucket.org/eigen/eigen/get/$eigen_version.tar.gz \
 && tar xf $eigen_version.tar.gz && cd eigen* \
 && mkdir build && cd build \
 && cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr \
 && ninja install \
 && cd / && rm -rf /eigen3
