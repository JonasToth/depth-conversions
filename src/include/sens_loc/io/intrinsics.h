#ifndef INTRINSICS_H_6SQKKYBV
#define INTRINSICS_H_6SQKKYBV

#include <istream>
#include <optional>
#include <sens_loc/camera_models/pinhole.h>
#include <sstream>

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
inline std::optional<camera_models::pinhole>
load_pinhole_intrinsic(std::istream &in) noexcept {
    if (!in.good())
        return std::nullopt;

    camera_models::pinhole p;
    double                 trash = 0.0;

    std::string line;
    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 1 with expected format 'w h'.
        ss >> p.w >> p.h;
        if (p.w <= 0 || p.h <= 0)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 2 with expected format 'fx 0.0 cx'.
        ss >> p.fx >> trash >> p.cx;
        if (p.fx <= 0. || p.cx <= 0.)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 3 with expected format '0.0 fy cy'.
        ss >> trash >> p.fy >> p.cy;
        if (p.fy <= 0. || p.cy <= 0.)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }
    return p;
}
}}  // namespace sens_loc::io

#endif /* end of include guard: INTRINSICS_H_6SQKKYBV */
