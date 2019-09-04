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

if ! ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" ; then
    print_error "Proper Data provided. Command should not fail"
    exit 1
fi

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data0-depth.png" \
    --horizontal bearing-0-horizontal.png \
    --vertical bearing-0-vertical.png \
    --diagonal bearing-0-diagonal.png \
    --anti-diagonal bearing-0-anti-diagonal.png
then
    print_error "Could not create all different bearing angle."
    exit 1
fi

if  [ ! -f bearing-0-horizontal.png ] || \
    [ ! -f bearing-0-vertical.png ] || \
    [ ! -f bearing-0-diagonal.png ] || \
    [ ! -f bearing-0-anti-diagonal.png ]; then
    print_error "Did not create expected output files."
    exit 1
fi
