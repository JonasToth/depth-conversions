#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if ! ${exe} \
    --input "sift-{}.feature" \
    --start 0 --end 1 \
    matching ; then
    print_error "Could not analyze sift matching"
    exit 1
fi

if ! ${exe} --input "orb-{}.feature" \
    --start 0 --end 1 \
    matching \
    --distance-norm HAMMING --no-crosscheck ; then
    print_error "Could not analyze orb matching"
    exit 1
fi

rm -f surf-matched-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    matching --match-output "surf-matched-{}.png" --original-images "../feature_extractor/flexion-{}.png" ; then
    print_error "Could not analyze sift matching and draw them"
    exit 1
fi
if  [ ! -f surf-matched-1.png ] ; then
    print_error "Did not create expected output files."
    exit 1
fi
