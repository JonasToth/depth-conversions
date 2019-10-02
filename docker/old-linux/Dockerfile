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
