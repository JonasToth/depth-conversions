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

if ! ${exe} \
    -i "data{}-depth.png" \
    -o "bilateral-distance-{}.png" \
    -s 0 -e 1 \
    bilateral --sigma-color 20. --distance 5 ; then
    print_error "Color-similarity and pixel-distance for filter must suffice"
    exit 1
fi

if ! ${exe} \
    -i "data{}-depth.png" \
    -o "bilateral-space-{}.png" \
    -s 0 -e 1 \
    bilateral --sigma-color 20. --sigma-space 10. ; then
    print_error "Color-similarity and pixel-distance for filter must suffice"
    exit 1
fi

print_info "Test successful!"
exit 0
