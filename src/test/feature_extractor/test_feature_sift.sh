#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

print_info "Cleaning old artifacts"
rm -f sift-*

set -v

if ! ${exe} \
    -i "flexion-{}.png" -o "sift-{}.feature" \
    -s 0 -e 1 \
    detector sift descriptor sift ; then
    print_error "Default SIFT-Detection did not work"
    exit 1
fi
if  [ ! -f sift-0.feature ] || \
    [ ! -f sift-1.feature ]; then
    print_error "Did not create expected output files."
    exit 1
fi
