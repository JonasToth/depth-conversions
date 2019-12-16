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
rm -f surf-*.png

set -v

if ! ${exe} \
    -i "flexion-{}.png" \
    -s 0 -e 1 \
    surf -o "surf-{}.png" ; then
    print_error "Default SURF-Detection did not work"
    exit 1
fi
if  [ ! -f surf-0.png ] || \
    [ ! -f surf-1.png ]; then
    print_error "Did not create expected output files."
    exit 1
fi

if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-extended-{}.png" \
   --extended-descriptor ; then
    print_error "Extended SURF descriptor not calculated"
    exit 1
fi
if  [ ! -f surf-extended-0.png ] || \
    [ ! -f surf-extended-1.png ]; then
    print_error "Did not create expected output files."
    exit 1
fi

if ! ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-upright-{}.png" \
   --upright ; then
    print_error "Upright unoriented SURF not calculated"
    exit 1
fi
if  [ ! -f surf-upright-0.png ] || \
    [ ! -f surf-upright-1.png ]; then
    print_error "Did not create expected output files."
    exit 1
fi

# Configuration range need to be validated
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --threshold -1 ; then
    print_error "Hessian Threshold too low accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --threshold 1501 ; then
    print_error "Hessian Threshold too high accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --n-octaves 0 ; then
    print_error "Number Octave Layers too low accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --n-octaves 11 ; then
    print_error "Number Octave Layers too high accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --octave-layers 0 ; then
    print_error "Octave Layers too low accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   surf -o "surf-bad-{}.png" \
   --octave-layers 11 ; then
    print_error "Octave Layers too high accepted"
    exit 1
fi
