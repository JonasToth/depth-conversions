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
rm -f akaze-*

set -v

if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   akaze -o "akaze-{}.akaze" ; then
    print_error "Default AKAZE-Detection did not work"
    exit 1
fi
if  [ ! -f akaze-0.akaze ] || \
    [ ! -f akaze-1.akaze ]; then
    print_error "Did not create expected output files."
    exit 1
fi
