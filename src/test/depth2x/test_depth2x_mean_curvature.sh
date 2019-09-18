#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

set -v

print_info "Clearing test directory from old test result files."
rm -f batch-mean-curv-*

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{}-depth.png" \
    -s 0 -e 1 \
    mean-curvature \
    --output "batch-mean-curv-{}.png"
then
    print_error "Could not create all mean-curv images."
    exit 1
fi

if  [ ! -f batch-mean-curv-0.png ] || \
    [ ! -f batch-mean-curv-1.png ]; then
    print_error "Did not create expected output files without leading 0."
    exit 1
fi

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{:03d}-depth.png" \
    --start 2 --end 4 \
    mean-curvature \
    --output "batch-mean-curv-{:03d}.png"
then
    print_error "Could not create all mean-curv images."
    exit 1
fi

if  [ ! -f batch-mean-curv-002.png ] || \
    [ ! -f batch-mean-curv-003.png ] || \
    [ ! -f batch-mean-curv-004.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi

print_info "Test successful!"
exit 0