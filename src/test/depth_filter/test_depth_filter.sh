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

if ${exe} ; then
    print_error "Not enough arguments calling not checked"
    exit 1
fi
if ${exe} \
    -i "data{}-depth.png" \
    -o "median-blur-{}.png" \
    -s 0 -e 1 \
    median-blur \
    median-blur --distance 5 ; then
    print_error "Applying filter twice is not supported"
    exit 1
fi

if ! ${exe} --version ; then
    print_error "Printing the version is required to work"
    exit 1
fi

print_info "Test successful!"
exit 0
