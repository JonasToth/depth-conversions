FROM alpine:edge

RUN mkdir /lib64 \
 && ln -s /lib/libc.musl-x86_64.so.1 /lib64/ld-linux-x86-64.so.2

RUN echo "I Bims" > /TestImage.txt
COPY build/src/apps/depth2x /usr/bin/depth2x

ENTRYPOINT ["/usr/bin/depth2x", ]
