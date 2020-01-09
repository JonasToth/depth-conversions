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
rm -f orb-*

set -v

if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   orb -o "orb-{}.feature" ; then
    print_error "Default ORB-Detection did not work"
    exit 1
fi
if  [ ! -f orb-0.feature ] || \
    [ ! -f orb-1.feature ]; then
    print_error "Did not create expected output files."
    exit 1
fi
if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   orb -o "orb-scale-{}.feature" \
   --scale-factor 1.5 ; then
    print_error "ORB Scale factor does not work"
    exit 1
fi
if  [ ! -f orb-scale-0.feature ] || \
    [ ! -f orb-scale-1.feature ]; then
    print_error "Did not create expected output files."
    exit 1
fi

# Configuration range need to be validated
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   orb -o "orb-bad-{}.feature" \
   --scale-factor 0.8 ; then
    print_error "Scale factor to low accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   orb -o "orb-bad-{}.feature" \
   --scale-factor 4.1 ; then
    print_error "Scale factor to high accepted"
    exit 1
fi
