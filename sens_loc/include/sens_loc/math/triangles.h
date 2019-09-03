#ifndef TRIANGLES_H_XWPDRVKT
#define TRIANGLES_H_XWPDRVKT

#include <cmath>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/math/constants.h>

namespace sens_loc { namespace math {
/// This function calculates the bearing angle between two neighbouring range
/// measurements.
/// \param phi is an angle in raidans. It is the delta between two light rays
/// that and symbolizes the resolution of the sensor. Required to be < 90°.
/// \param cos_phi == std::cos(phi) as an optimization.
/// \param d_i reference depth and > 0
/// \param d_j prior depth and > 0
///
/// The bearing angle is angle between the ray to 'd_i' and the connecting line
/// from 'd_i' to 'd_j'.
/// It is bigger 0° and smaller 180° due to triangle constraints.
template <typename Real>
inline Real bearing_angle(const Real b, const Real c,
                          const Real cos_alpha) noexcept {
    Expects(b > 0.);
    Expects(c > 0.);
    // => alpha is smaller 90°
    Expects(cos_alpha > 0.);
    // => alpha is bigger 0°
    Expects(cos_alpha < 1.);

    const Real nom = 2. * b * b - 2. * b * c * cos_alpha;
    const Real den = 2. * b * std::sqrt(b * b + c * c - 2. * b * c * cos_alpha);
    const Real ratio = nom / den;

    Ensures(ratio > -1.);
    Ensures(ratio < +1.);

    const Real result = std::acos(ratio);

    Ensures(result > 0.);
    Ensures(result < math::pi<Real>);

    return result;
}
}}  // namespace sens_loc::math

#endif /* end of include guard: TRIANGLES_H_XWPDRVKT */
