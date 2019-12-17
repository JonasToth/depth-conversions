#ifndef UTILITY_H_SAL9APRW
#define UTILITY_H_SAL9APRW

#include <sens_loc/camera_models/concepts.h>
#include <sens_loc/math/coordinate.h>
#include <type_traits>

namespace sens_loc::camera_models {

/// This function calculates the angle of the rays between two pixels.
/// \param calibration projection model for a specific sensor
/// \param p1,p2 non-negative pixel coordinates smaller \c w and \c h.
/// \returns radians of the angle between the two lightrays.
template <template <typename> typename Intrinsic,
          typename Real   = float,
          typename Number = int>
Real phi(const Intrinsic<Real>&           calibration,
         const math::pixel_coord<Number>& p1,
         const math::pixel_coord<Number>& p2) noexcept {
    static_assert(is_intrinsic_v<Intrinsic, Real>);
    static_assert(std::is_floating_point_v<Real>);
    static_assert(std::is_arithmetic_v<Number>);

    Expects(p1.u() >= 0);
    Expects(p1.u() < calibration.w());

    Expects(p1.v() >= 0);
    Expects(p1.v() < calibration.h());

    Expects(p2.u() >= 0);
    Expects(p2.u() < calibration.w());

    Expects(p2.v() >= 0);
    Expects(p2.v() < calibration.h());

    const auto s1      = calibration.pixel_to_sphere(p1);
    const auto s2      = calibration.pixel_to_sphere(p2);
    const auto cos_phi = s1.dot(s2);

    Ensures(cos_phi > -1.);
    Ensures(cos_phi < +1.);

    const auto angle = std::acos(cos_phi);
    return angle;
}

}  // namespace sens_loc::camera_models

#endif /* end of include guard: UTILITY_H_SAL9APRW */
