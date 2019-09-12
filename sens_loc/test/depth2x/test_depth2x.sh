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

if ${exe} ; then
    print_error "Not enough arguments calling not checked"
    exit 1
fi

if ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4 ; then
    print_error "Subcommand is required"
    exit 1
fi

if ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4 bearing; then
    print_error "One output-pattern required. Failure to enforce that"
    exit 1
fi

print_info "Test successful!"
exit 0
