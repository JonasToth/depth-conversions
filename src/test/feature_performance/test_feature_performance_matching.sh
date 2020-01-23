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
    matching ; then
    print_error "Could not analyze sift matching"
    exit 1
fi

if ! ${exe} --input "orb-{}.feature" \
    --start 0 --end 1 \
    matching \
    --distance-norm HAMMING --no-crosscheck ; then
    print_error "Could not analyze orb matching"
    exit 1
fi