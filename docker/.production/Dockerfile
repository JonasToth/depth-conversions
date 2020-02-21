FROM bitnami/minideb:latest
LABEL maintainer="jonas.toth@student.tu-freiberg.de"

COPY depth2x /usr/bin/depth2x
COPY depth_filter /usr/bin/depth_filter
COPY feature_extractor /usr/bin/feature_extractor
COPY keypoint_plotter /usr/bin/keypoint_plotter
COPY feature_performance /usr/bin/feature_performance

RUN apt-get update \
 && apt-get install -y \
    python3 \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
COPY transformation_step.py /usr/bin/transformation_step.py
