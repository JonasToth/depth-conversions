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
rm -f batch-bearing-*

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

if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{}-depth.png" \
    -s 0 -e 1 \
    bearing \
    --horizontal "batch-bearing-{}-horizontal.png" \
    --vertical "batch-bearing-{}-vertical.png" \
    --diagonal "batch-bearing-{}-diagonal.png" \
    --anti-diagonal "batch-bearing-{}-anti-diagonal.png"
then
    print_error "Could not create all different bearing angle."
    exit 1
fi

if  [ ! -f batch-bearing-0-horizontal.png ] || \
    [ ! -f batch-bearing-0-vertical.png ] || \
    [ ! -f batch-bearing-0-diagonal.png ] || \
    [ ! -f batch-bearing-0-anti-diagonal.png ] || \
    [ ! -f batch-bearing-1-horizontal.png ] || \
    [ ! -f batch-bearing-1-vertical.png ] || \
    [ ! -f batch-bearing-1-diagonal.png ] || \
    [ ! -f batch-bearing-1-anti-diagonal.png ]; then
    print_error "Did not create expected output files without leading 0."
    exit 1
fi


if ! ${exe} -c "kinect_intrinsic.txt" \
    -i "data{:03d}-depth.png" \
    --start 2 --end 4 \
    bearing \
    --horizontal "batch-bearing-{:03d}-horizontal.png" \
    --vertical "batch-bearing-{:03d}-vertical.png" \
    --diagonal "batch-bearing-{:03d}-diagonal.png" \
    --anti-diagonal "batch-bearing-{:03d}-anti-diagonal.png"
then
    print_error "Could not create all different bearing angle."
    exit 1
fi

if  [ ! -f batch-bearing-002-horizontal.png ] || \
    [ ! -f batch-bearing-002-vertical.png ] || \
    [ ! -f batch-bearing-002-diagonal.png ] || \
    [ ! -f batch-bearing-002-anti-diagonal.png ] || \
    [ ! -f batch-bearing-003-horizontal.png ] || \
    [ ! -f batch-bearing-003-vertical.png ] || \
    [ ! -f batch-bearing-003-diagonal.png ] || \
    [ ! -f batch-bearing-003-anti-diagonal.png ] || \
    [ ! -f batch-bearing-004-horizontal.png ] || \
    [ ! -f batch-bearing-004-vertical.png ] || \
    [ ! -f batch-bearing-004-diagonal.png ] || \
    [ ! -f batch-bearing-004-anti-diagonal.png ]; then
    print_error "Did not create expected output files with leading 0."
    exit 1
fi

print_info "Test successful!"
