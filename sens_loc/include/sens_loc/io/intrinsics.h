#ifndef INTRINSICS_H_6SQKKYBV
#define INTRINSICS_H_6SQKKYBV

#include <istream>
#include <optional>
#include <sens_loc/camera_models/pinhole.h>

namespace sens_loc { namespace io {

/// Load the intrinsic parameters from an input stream (e.g. a filestream).
///
/// The expected format is as follows (each value is a floating point number):
/// ```
/// w h
/// <fx> 0.0  <cx>
/// 0.0  <fy> <cy>
/// 0.0  0.0  1.0
/// ```
/// @note values in brackets are optional.
std::optional<camera_models::pinhole> load_pinhole_intrinsic(std::istream &in);

}}  // namespace sens_loc::io

#endif /* end of include guard: INTRINSICS_H_6SQKKYBV */
