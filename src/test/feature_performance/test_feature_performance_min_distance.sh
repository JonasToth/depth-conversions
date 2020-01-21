#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if ! ${exe} --input "sift-{}.feature" \
    --start 0 --end 1 \
    min-distance --norm L2 ; then
    print_error "Could not analyze sift features under L2 norm"
    exit 1
fi

if ! ${exe} --input "orb-{}.feature" \
    --start 0 --end 1 \
    min-distance --norm HAMMING ; then
    print_error "Could not analyze orb features under Hamming norm"
    exit 1
fi
