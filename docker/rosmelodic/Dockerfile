FROM ros:melodic-perception
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
 && pip3 install --upgrade pip \
 && pip install cmake
