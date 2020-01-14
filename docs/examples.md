# Usage examples

## depth2x

This tool does depth-map conversions to different derived images that are more
useful for localization tasks.
Right now the input is always assumed to be created with a pinhole-camera
without any distortion.
This means that preprocessing of the images might be necessary. With an
intrinsic calibration of the camera the images can be undistorted and then used
as input.
The `fx, fy, cx, cy` then make up the full intrinsic and can be supplied to the
command.

### Intrinsic files

Intrinsic files follow the following conversion, describing the camera matrix.

```bash
$ cat intrinsic.txt  # example intrinsic for a kinect pinhole image
> 960 540                       # == <width> <height>
> 519.226 0.000000 479.462      # == <fx> ignored <cx>
> 0.000000 522.23 272.737       # == ignored <fy> <cy>
> 0.000000 0.000000 1.000000    # this line is ignored
```

### Expected images

The tools is expected to work with 16-bit PNG files as depth images.
Providing a different file format results in a failure of the tool.

The output is 16-bit PNG as well.

### General structure

All commands are for batch-conversion tasks that are executed in parallel.

```bash
$ depth2x <target-format> \
        --calibration <intrinsic-file> \
        --input <python-like pattern with integer substitution> \
        --output <python-like pattern with integer substitution> \
        --start <index to start with> \
        --end <index to end with, inclusive!>
> Run-Log and potential errors
```

Please consult the help messages for the command itself as well as for each
subcommand.

### Examplaric creation of flexion images

The following command convert orthographic depth-maps into flexion images.

```bash
$ depth2x flexion \
    --calibration intrinsic.txt \
    --input depth_{:04d}.png \
    --start 0 \
    --end 100 \
    --output flexion_{:04d}.png
> Run-Log and potential errors
```

### Examplaric scaling of depth-images to better see the content

The following command convert orthographic depth-maps into flexion images.

```bash
$ depth2x scale \
          --input depth_{:04d}.png \
          --output scaled_{:04d}.png \
          --start 0 \
          --end 100 \
          --factor 8.0
> Run-Log and potential errors
```

## depth_filter

This tool can apply different filters on the depth image. This is necessary
as sensor input has different levels of noise. Smoothing this noise results in
sharper images that in turn might work better for registration.

This process does in general not require an intrinsic as it works only on the
image itself. The filters are implemented in
[OpenCV](https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html).

Please note that the filters have different computational complexity that
depends on the parameters as well. Application is a trade-off.

```bash
$ depth_filter median-blur --input depth_{:04d}.png \
                           --output filtered_{:04d}.png \
                           --start 0 \
                           --end 100 \
                           --distance 5
> Run-Log and potential errors
```

## feature_extractor

This tools reads in the generated and/or filtered feature images and runs
different keypoint detectors and feature descriptors of these images.
The input is expected to be **16 bit grayscale png**.

The executable wraps only the
[OpenCV Features Module](https://docs.opencv.org/master/da/d9b/group__features2d.html)
and their
[OpenCV XFeatures](https://docs.opencv.org/master/d1/db4/group__xfeatures2d.html).
Not every feature detector and descriptor is used, but every one could be
added.

The output are `.feature` files, that are written with
[OpenCV Filestorage](https://docs.opencv.org/master/da/d56/classcv_1_1FileStorage.html).
The output is `YAML`. Appending `.gz` to the file-name will compress those files
as well.

Each file consists of the nodes `source_path` (the provided path while feature
extractions), `keypoints` (array of the detected keypoints) and `descriptors`
(matrix where each row is the extracted feature descriptor and for the n'th
keypoint). Optionally the used detector configuration is written, but that
depends on the implementation support for that.

```
$ feature_extractor --input "flexion-{}.png" \
                    --start 0                \
                    --end 100                \
                    akaze --output akaza-{:04d}.feature[.gz]
> Run-Log and potential Errors
> .feature-files are in the local directory
```

## keypoint_plotter

This tool is a little utility to plot the detected keypoints on the actual
images. It does no further processing, but only reads in files generated
with `feature_extractor` and plots them.
It tries to use the `source_path`, but that can be overwritten with the
`--original-file` option.

```
$ keypoint_plotter --input akaze-{}.feature[.gz] \
                   --output keypoints-{}.png     \
                   --start 0 --end 100
> Uses the paths from the feature file to plot the keypoints on.

$ keypoint_plotter --input akaze-{:04d}.feature[.gz] \
                   --original-file depth-{}.png      \
                   --output keypoints-depth-{}.png   \
                   --start 0 --end 100 ;
> Overwrite the 'source_path' do use a different file then specified in the
> feature file.
```
