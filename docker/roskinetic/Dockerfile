FROM ros:kinetic-perception

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

RUN mkdir -p /eigen3 && cd /eigen3 \
 && wget --quiet http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz \
 && tar xf 3.3.7.tar.gz && cd eigen* \
 && mkdir build && cd build \
 && cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr \
 && ninja install \
 && cd / && rm -rf /eigen3
