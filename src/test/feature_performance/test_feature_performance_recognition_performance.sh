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
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" ; then
    print_error "Could not calculate precision and recall"
    exit 1
fi

rm -f recognition.stat
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    --output recognition.stat \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" ; then
    print_error "Could not calculate precision and recall"
    exit 1
fi
if [ ! -f recognition.stat ] ; then
    print_error "Did not create statistic file with recognition performance"
    exit 1
fi

print_info "Testing Plotting the backprojections of keypoints"
rm -f backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --backprojection "backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Could not calculate precision and recall and plot the reprojection"
    exit 1
fi
if [ ! -f backprojected-1.png ] ; then
    print_error "Did not create expected unmasked output files."
    exit 1
fi

print_info "Testing masks for the field of view"
rm -f masked-backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --mask "distortion_mask.png" \
    --backprojection "masked-backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Could not calculate precision and recall and plot the reprojection using a mask to remove keypoints outside of field of view"
    exit 1
fi
if [ ! -f masked-backprojected-1.png ] ; then
    print_error "Did not create expected masked backprojected output files."
    exit 1
fi

print_info "Testing bad masks"
rm -f unmasked-backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --mask "distortion_mask_bad.png" \
    --backprojection "unmasked-backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Could not calculate precision and recall and plot the reprojection, ignoring a bad mask"
    exit 1
fi
if [ ! -f unmasked-backprojected-1.png ] ; then
    print_error "Did not create expected unmasked output files."
    exit 1
fi
rm -f unmasked-backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --mask "does-not-exist-distortion_mask_bad.png" \
    --backprojection "unmasked-backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Could not calculate precision and recall and plot the reprojection, ignoring an non-existing mask"
    exit 1
fi
if [ ! -f unmasked-backprojected-1.png ] ; then
    print_error "Did not create expected unmasked output files."
    exit 1
fi

print_info "Testing bad file input for depth images"
if ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "does-not-exist-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" ; then
    print_error "Did not signal failure when depth images can not be loaded"
    exit 1
fi

print_info "Testing bad file input for poses"
if ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "does-not-exist-pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" ; then
    print_error "Did not signal failure when the poses can not be loaded"
    exit 1
fi

print_info "Testing bad original files input for backprojection"
rm -f backprojected-1.png
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --backprojection "backprojected-{}.png" \
    --orig-images "does-not-exist-flexion-{}.png" ; then
    print_error "Ignoring that backprojection does not work, but analysis does not done properly"
    exit 1
fi
if [ -f backprojected-1.png ] ; then
    print_error "Did not create expected unmasked output files."
    exit 1
fi

print_info "Testing bad path for backprojection output"
rm -f backprojected-1.jpg
if ! ${exe} \
    --input "surf-1-octave-{}.feature.gz" \
    --start 0 --end 1 \
    recognition-performance \
    --depth-image "filtered-{}.png" \
    --pose-file "pose-{}.pose" \
    --intrinsic "kinect_intrinsic.txt" \
    --match-norm "L2" \
    --backprojection "directory-does-not-exist/backprojected-{}.png" \
    --orig-images "flexion-{}.png" ; then
    print_error "Ignoring that backprojection does not work, because the files can not be written, but analysis work is not done properly"
    exit 1
fi
if [ -f directory-does-not-exist/backprojected-{}.png ] ; then
    print_error "Created file which is not expected."
    exit 1
fi
