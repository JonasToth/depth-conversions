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
