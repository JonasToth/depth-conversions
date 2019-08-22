#ifndef INTRINSICS_H_6SQKKYBV
#define INTRINSICS_H_6SQKKYBV

#include <optional>
#include <istream>

namespace sens_loc { namespace io {

/// This struct is a container holding parameters for the pinhole camera model.
struct pinhole_parameters {
    double fx;  //< x-coordinate of focal length
    double fy;  //< y-corrdiante of focal length
    double cx;  //< x-coordinate of image center
    double cy;  //< y-coordinate of image center

#if 0
    double p1 = 0.0; //< first order tangential distortion coefficient
    double p2 = 0.0; //< second order tangential distortion coefficient

    double k1 = 0.0; //< first order radial distortion coefficient
    double k2 = 0.0; //< second order radial distortion coefficient
    double k3 = 0.0; //< third order radial distortion coefficient
#endif
};

/// Load the intrinsic parameters from an input stream (e.g. a filestream).
///
/// The expected format is as follows (each value is a floating point number):
/// ```
/// <fx> 0.0  <cx>
/// 0.0  <fy> <cy>
/// 0.0  0.0  1.0
/// ```
/// @note values in brackets are optional.
std::optional<pinhole_parameters> load_pinhole_intrinsic(std::istream& in);

}}  // namespace sens_loc::io

#endif /* end of include guard: INTRINSICS_H_6SQKKYBV */
