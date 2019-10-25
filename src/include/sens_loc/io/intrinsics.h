#ifndef INTRINSICS_H_6SQKKYBV
#define INTRINSICS_H_6SQKKYBV

#include <istream>
#include <optional>
#include <sens_loc/camera_models/equirectangular.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sstream>

namespace sens_loc {

///
template <typename Real, template <typename> typename Intrinsic>
class io {
  public:
    static std::optional<Intrinsic<Real>>
    load_intrinsic(std::istream &) noexcept {
        return std::nullopt;
    }
};

template <typename Real>
class io<Real, camera_models::pinhole> {
  public:
    /// Load the pinhole intrinsic parameters from an input stream
    /// (e.g. a filestream).
    ///
    /// The expected format is as follows (each value is a floating point
    /// number):
    /// ```
    /// w h
    /// <fx> 0.0  <cx>
    /// 0.0  <fy> <cy>
    /// [0.0  0.0  1.0]
    /// ```
    /// \note Each value is expected to be positive and negative values are
    /// treated as failure. \note Values in brackets are optional. \warning The
    /// function is sensitive to UNIX-vs-MS line-endings. \returns \c
    /// std::optional<> with proper parameters on success, otherwise it contains
    /// \c None.
    static std::optional<camera_models::pinhole<Real>>
    load_intrinsic(std::istream &in) noexcept {
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
            if (ss.fail() || w <= 0 || h <= 0)
                return std::nullopt;

            if (ss.rdstate() != std::ios_base::eofbit)
                return std::nullopt;
        }

        {
            std::getline(in, line);
            std::istringstream ss{line};
            // Parsing line 2 with expected format 'fx 0.0 cx'.
            ss >> fx >> trash >> cx;
            if (ss.fail() || fx <= 0. || cx <= 0.)
                return std::nullopt;

            if (ss.rdstate() != std::ios_base::eofbit)
                return std::nullopt;
        }

        {
            std::getline(in, line);
            std::istringstream ss{line};
            // Parsing line 3 with expected format '0.0 fy cy'.
            ss >> trash >> fy >> cy;
            if (ss.fail() || fy <= 0. || cy <= 0.)
                return std::nullopt;

            if (ss.rdstate() != std::ios_base::eofbit)
                return std::nullopt;
        }
        return camera_models::pinhole<Real>(w, h, fx, fy, cx, cy);
    }
};


template <typename Real>
class io<Real, camera_models::equirectangular> {
  public:
    /// Load the equirectangular intrinsic parameters from an input stream
    /// (e.g. a filestream).
    ///
    /// The expected format is as follows (each value is a floating point
    /// number):
    /// ```
    /// w h
    /// [<theta_min> <theta_max>]
    /// ```
    /// \note values for \f$\theta\f$ are in radians and need to be smaller
    /// \f$\pi\f$ and \f$\theta_{min} < \theta_{max}\f$.
    /// \note Each value is expected to be positive and negative values are
    /// treated as failure. \note values in brackets are optional. \note in
    /// horizontal direction the equirectangular image is always 360°
    /// vertical range the resolution in controllable \warning The function is
    /// sensitive to UNIX-vs-MS line-endings. \returns \c std::optional<> with
    /// proper parameters on success, otherwise it contains \c None.
    static std::optional<camera_models::equirectangular<Real>>
    load_intrinsic(std::istream &in) noexcept {
        if (!in.good())
            return std::nullopt;

        int  w         = 0;
        int  h         = 0;
        Real theta_min = Real(0.);
        Real theta_max = math::pi<Real>;

        std::string line;
        {
            std::getline(in, line);
            std::istringstream ss{line};
            // Parsing line 1 with expected format 'w h'.
            ss >> w >> h;
            if (ss.fail() || w <= 0 || h <= 0)
                return std::nullopt;
        }

        if (!in.eof()) {
            std::getline(in, line);
            std::istringstream ss{line};
            // Parsing line 2 with expected format '[<theta_min> <theta_max>]'.
            ss >> theta_min >> theta_max;
            if (ss.fail() || theta_min <= Real(0.) || theta_max <= Real(0.) ||
                theta_min >= theta_max || theta_min >= math::pi<Real> ||
                theta_max >= math::pi<Real>)
                return std::nullopt;
        }

        return camera_models::equirectangular<Real>(w, h,
                                                    {theta_min, theta_max});
    }
};
}  // namespace sens_loc

#endif /* end of include guard: INTRINSICS_H_6SQKKYBV */
