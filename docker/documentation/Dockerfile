FROM ubuntu:18.04
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update \
 && apt-get dist-upgrade -y \
 && apt-get install -y \
    doxygen \
    graphviz
