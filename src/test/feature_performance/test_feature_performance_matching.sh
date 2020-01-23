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
    --input "sift-{}.feature" \
    --start 0 --end 1 \
    matching \
    --image-width 960 --image-height 540 ; then
    print_error "Could not analyze sift keypoints"
    exit 1
fi

if ! ${exe} --input "orb-{}.feature" \
    --start 0 --end 1 \
    matching \
    --image-width 960 --image-height 540 ; then
    print_error "Could not analyze orb keypoints"
    exit 1
fi
