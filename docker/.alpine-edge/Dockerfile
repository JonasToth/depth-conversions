FROM alpine:edge
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

COPY depth2x /usr/bin/depth2x
COPY depth_filter /usr/bin/depth_filter
COPY feature_extractor /usr/bin/feature_extractor
COPY keypoint_plotter /usr/bin/keypoint_plotter
COPY feature_performance /usr/bin/feature_performance

RUN apk add python3
COPY transformation_step.py /usr/bin/transformation_step.py
