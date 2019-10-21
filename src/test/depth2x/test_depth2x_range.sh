#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if grep --silent "Precise Pangolin" /etc/os-release ; then
    print_warning "Skipping Tests on old linux - See #8 for more information!"
    exit 0
fi

set -v

print_info "Clearing test directory from old test result files."
rm -f batch-range-*

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{}-depth.png" \
    -s 0 -e 1 \
    range \
    --output "batch-range-{}.png"
then
    print_error "Could not create all range images."
    exit 1
fi

if  [ ! -f batch-range-0.png ] || \
    [ ! -f batch-range-1.png ]; then
    print_error "Did not create expected output files without leading 0."
    exit 1
fi

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{:03d}-depth.png" \
    --start 2 --end 4 \
    range \
    --output "batch-range-{:03d}.png"
then
    print_error "Could not create all range images."
    exit 1
fi

if  [ ! -f batch-range-002.png ] || \
    [ ! -f batch-range-003.png ] || \
    [ ! -f batch-range-004.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi

# Try out if the range images are useable as input for other conversions.

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "batch-range-{:03d}.png" \
    --start 2 --end 4 \
    --type pinhole-range \
    flexion \
    --output "batch-flexion-from-range-{:03d}.png"
then
    print_error "Could not create all flexion images from range images."
    exit 1
fi

if  [ ! -f batch-flexion-from-range-002.png ] || \
    [ ! -f batch-flexion-from-range-003.png ] || \
    [ ! -f batch-flexion-from-range-004.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi

print_info "Test successful!"
exit 0
