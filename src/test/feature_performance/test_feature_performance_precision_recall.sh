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
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    precision-recall \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" ; then
    print_error "Could not calculate precision and recall"
    exit 1
fi

rm -f backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    precision-recall \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --backprojection "backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Could not calculate precision and recall and plot the reprojection"
    exit 1
fi
if [ ! -f backprojected-1.png ] ; then
    print_error "Did not create expected output files."
    exit 1
fi
