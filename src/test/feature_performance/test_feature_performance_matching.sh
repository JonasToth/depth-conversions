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

print_info "Test for bad output-path after matching"
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    matching --match-output "does-not-exist/surf-matched-{}.png" --original-images "../feature_extractor/flexion-{}.png" ; then
    print_error "Did not ignore failure of writing match-output"
    exit 1
fi
if  [ -f does-not-exist/surf-matched-1.png ] ; then
    print_error "Unexpected file created."
    exit 1
fi

print_info "Test for non-existing features."
if ${exe} \
    --input "does-not-exist-{}.feature.gz" \
    --start 0 --end 1 \
    matching ; then
    print_error "Did not signal failure for non-existing features"
    exit 1
fi
