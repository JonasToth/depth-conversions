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
rm -f batch-scale-*

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{}-depth.png" \
    -s 0 -e 1 \
    scale \
    --output "batch-scale-{}.png" \
    --factor "8.0"
then
    print_error "Could not create all scaled images."
    exit 1
fi

if  [ ! -f batch-scale-0.png ] || \
    [ ! -f batch-scale-1.png ]; then
    print_error "Did not create expected output files without leading 0."
    exit 1
fi

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{:03d}-depth.png" \
    --start 2 --end 4 \
    scale \
    --output "batch-scale-{:03d}.png" \
    --delta "5000.0"
then
    print_error "Could not create all scale images."
    exit 1
fi

if  [ ! -f batch-scale-002.png ] || \
    [ ! -f batch-scale-003.png ] || \
    [ ! -f batch-scale-004.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi

# Deliberatly convert more then existing files to cover this error path
# as well.
if ${exe} -c "kinect_intrinsic.txt" \
    -i "data{:03d}-depth.png" \
    --start 2 --end 10 \
    scale \
    --output "batch-scale-{:03d}.png" \
    --delta "5000.0"
then
    print_error "Batch process needed to fail because the index range was invalid"
    exit 1
fi
print_info "Test successful!"
exit 0
