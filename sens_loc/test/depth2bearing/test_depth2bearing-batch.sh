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

if ! ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4; then
    print_error "Proper Data provided. Command should not fail"
    exit 1
fi

if ! ${exe} -c "kinect_intrinsic.txt" -i "data{}-depth.png" \
    --horizontal "bearing-{}-horizontal.png" \
    --vertical "bearing-{}-vertical.png" \
    --diagonal "bearing-{}-diagonal.png" \
    --anti-diagonal "bearing-{}-anti-diagonal.png" \
    -s 0 -e 1
then
    print_error "Could not create all different bearing angle."
    exit 1
fi

if [ ! -f bearing-0-horizontal.png ] || [ ! -f bearing-0-vertical.png ] || \
   [ ! -f bearing-0-diagonal.png ] || [ ! -f bearing-0-anti-diagonal.png ] || \
   [ ! -f bearing-1-horizontal.png ] || [ ! -f bearing-1-vertical.png ] || \
   [ ! -f bearing-1-diagonal.png ] || [ ! -f bearing-1-anti-diagonal.png ]; then
    print_error "Did not create expected output files without leading 0."
    exit 1
fi


if ! ${exe} -c "kinect_intrinsic.txt" -i "data{:03d}-depth.png" \
    --horizontal "bearing-{:03d}-horizontal.png" \
    --vertical "bearing-{:03d}-vertical.png" \
    --diagonal "bearing-{:03d}-diagonal.png" \
    --anti-diagonal "bearing-{:03d}-anti-diagonal.png" \
    --start 2 --end 4
then
    print_error "Could not create all different bearing angle."
    exit 1
fi

if [ ! -f bearing-002-horizontal.png ] || [ ! -f bearing-002-vertical.png ] || \
   [ ! -f bearing-002-diagonal.png ]   || [ ! -f bearing-002-anti-diagonal.png ] || \
   [ ! -f bearing-003-horizontal.png ] || [ ! -f bearing-003-vertical.png ] || \
   [ ! -f bearing-003-diagonal.png ] || [ ! -f bearing-003-anti-diagonal.png ] || \
   [ ! -f bearing-004-horizontal.png ] || [ ! -f bearing-004-vertical.png ] || \
   [ ! -f bearing-004-diagonal.png ] || [ ! -f bearing-004-anti-diagonal.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi
