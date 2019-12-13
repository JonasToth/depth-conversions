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

if ! ${exe} \
    -i "flexion-{}.png" \
    -s 0 -e 1 \
    sift -o "sift-{}.png" ; then
    print_error "Default SIFT-Detection did not work"
    exit 1
fi

# Invalid arguments
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1  sift ; then
    print_error "SIFT requires output path"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --feature-count -1 ; then
    print_error "Feature-Count negative was accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --feature-count 10001 ; then
    print_error "Feature-Count too big was accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --octave-layers 0 ; then
    print_error "Layer-Count zero was accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --octave-layers 11 ; then
    print_error "Layer-Count too big was accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --contrast-threshold -0.1 ; then
    print_error "Negative Contrast Threshold Accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --contrast-threshold 1.1 ; then
    print_error "Too Big Contrast Threshold Accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --edge-threshold -0.1 ; then
    print_error "Negative Edge Threshold Accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --edge-threshold 101. ; then
    print_error "Too Big Edge Threshold Accepted"
    exit 1
fi

if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --sigma -0.1 ; then
    print_error "Negative Sigma Accepted"
    exit 1
fi
if ${exe} \
   -i "flexion-{}.png" \
   -s 0 -e 1 \
   sift -o "sift-bad-{}.png" \
   --sigma 11. ; then
    print_error "Too Big Sigma Accepted"
    exit 1
fi
