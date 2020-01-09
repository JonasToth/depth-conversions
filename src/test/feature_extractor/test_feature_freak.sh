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
rm -f agastfreak-*

set -v

if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   agastfreak -o "agastfreak-{}.feature" ; then
    print_error "Default FREAK-Detection did not work"
    exit 1
fi
if  [ ! -f agastfreak-0.feature ] || \
    [ ! -f agastfreak-1.feature ]; then
    print_error "Did not create expected output files."
    exit 1
fi
