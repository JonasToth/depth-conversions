#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if ! ${exe} \
    --input sift-{}.sift \
    --output keypoints-{}.png \
    --start 0 --end 1 ; then
    print_error "Could not plot keypoint in selfincluded feature-file (8bit gray, 16bit gray)"
    exit 1
fi
if [ ! -f keypoints-0.png ] || \
   [ ! -f keypoints-1.png ] ; then
    print_error "Expected files not created"
    exit 1
fi
