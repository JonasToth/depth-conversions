FROM alpine:edge
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

RUN apk update \
 && apk add bash \
            build-base \
            ninja \
            cmake \
            gcc g++ \
            linux-headers \
            wget \
            unzip \
            rsync \
 && mkdir -p /opencv/build_cv && cd /opencv \
 && wget --quiet https://github.com/opencv/opencv/archive/4.1.1.zip \
 && unzip 4.1.1.zip && rm 4.1.1.zip \
 && wget --quiet https://github.com/opencv/opencv_contrib/archive/4.1.1.zip \
 && unzip 4.1.1.zip && rm 4.1.1.zip \
 && cd /opencv/build_cv \
 && CC=gcc CXX=g++ cmake /opencv/opencv-4.1.1/ \
    -G Ninja \
    -DCMAKE_EXE_LINKER_FLAGS="-static-libstdc++ -static-libgcc" \
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
