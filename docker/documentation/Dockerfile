FROM ubuntu:18.04
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

WORKDIR /
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update \
 && apt-get dist-upgrade -y \
 && apt-get install -y \
    graphviz \
    wget \
 && wget http://doxygen.nl/files/doxygen-1.8.16.linux.bin.tar.gz \
 && tar xf doxygen-1.8.16.linux.bin.tar.gz \
 && cd doxygen-1.8.16/bin \
 && mv * /usr/local/bin \
 && rm -rf doxygen-1.8.16/
