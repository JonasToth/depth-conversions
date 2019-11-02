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

rm -f median-blur-*

if ! ${exe} \
    -i "data{}-depth.png" \
    -o "median-blur-{}.png" \
    -s 0 -e 1 \
    median-blur --distance 5 ; then
    print_error "Median blur filter needs to work"
    exit 1
fi


if ${exe} \
    -i "data{}-depth.png" \
    -o "median-blur-{}.png" \
    -s 0 -e 1 \
    median-blur --distance 11 ; then
    print_error "Distance is of invalid value"
    exit 1
fi

print_info "Test successful!"
exit 0
