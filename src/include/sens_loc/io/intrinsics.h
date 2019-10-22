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
/// [0.0  0.0  1.0]
/// ```
/// \note Each value is expected to be positive and negative values are treated
/// as failure.
/// \note Values in brackets are optional.
/// \warning The function is sensitive to UNIX-vs-MS line-endings.
/// \returns \c std::optional<> with proper parameters on success, otherwise
/// it contains \c None.
template <typename Real>
inline std::optional<camera_models::pinhole<Real>>
load_pinhole_intrinsic(std::istream &in) noexcept {
    if (!in.good())
        return std::nullopt;

    int  w     = 0;
    int  h     = 0;
    Real fx    = 0.;
    Real fy    = 0.;
    Real cx    = 0.;
    Real cy    = 0.;
    Real trash = 0.0;

    std::string line;
    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 1 with expected format 'w h'.
        ss >> w >> h;
        if (w <= 0 || h <= 0)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 2 with expected format 'fx 0.0 cx'.
        ss >> fx >> trash >> cx;
        if (fx <= 0. || cx <= 0.)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 3 with expected format '0.0 fy cy'.
        ss >> trash >> fy >> cy;
        if (fy <= 0. || cy <= 0.)
            return std::nullopt;

        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }
    return camera_models::pinhole<Real>(w, h, fx, fy, cx, cy);
}
}}  // namespace sens_loc::io

#endif /* end of include guard: INTRINSICS_H_6SQKKYBV */
