#ifndef TRIANGLES_H_XWPDRVKT
#define TRIANGLES_H_XWPDRVKT

#include <cmath>
#include <gsl/gsl>
#include <iostream>
#include <limits>
#include <sens_loc/math/constants.h>
#include <type_traits>

namespace sens_loc { namespace math {

/// This function calculates the bearing angle between two neighbouring range
/// measurements.
/// \param b reference depth and > 0
/// \param c prior depth and > 0
/// \param cos_alpha == std::cos(alpha) of the angle between two measurements
/// \returns bearing angle in radians
/// \pre b and c and positive values
/// \pre \f$0 < \cos \alpha < 1\f$
/// \post \f$0 < result < \pi\f$
///
/// The bearing angle is angle between the ray to 'b' and the connecting line
/// from 'c' to 'b'.
/// It is bigger 0° and smaller 180° due to triangle constraints.
template <typename Real>
inline Real
bearing_angle(const Real b, const Real c, const Real cos_alpha) noexcept {
    static_assert(std::is_arithmetic_v<Real>);

    Expects(b > 0.);
    Expects(c > 0.);
    // => alpha is smaller 90°
    Expects(cos_alpha > 0.);
    // => alpha is bigger 0°
    Expects(cos_alpha < 1.);

    const Real nom = b - c * cos_alpha;
    const Real den = std::sqrt(b * b + c * c - 2. * b * c * cos_alpha);
    Ensures(den != 0.);

    Real ratio = nom / den;

    /// Note: Because of inaccuracy of floating point operations it is possible
    /// to get abs(ratio) == 1. This is not an error in the implementation
    /// but rather an numerical artifact. To not hit the post-conditions
    /// it is ok to subtract a tiny epsilon.
    /// The problem only occured with 'float' but did not occur with 'double'.
    if (ratio >= 1.) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        ratio = 1. - Real(10.) * std::numeric_limits<Real>::epsilon();
    }
    if (ratio <= -1.) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        ratio = -1. + Real(10.) * std::numeric_limits<Real>::epsilon();
    }

    Ensures(ratio > -1.);
    Ensures(ratio < +1.);

    const Real result = std::acos(ratio);

    Ensures(result > 0.);
    Ensures(result < math::pi<Real>);

    return result;
}

/// This function implements the bearing angle with formula from Lin, which is
/// incorrect.
/// This implementation is here to verify the difference in the results
/// and testing covers that as well.
template <typename Real = float>
inline Real reference_lin_bearing_angle(const Real b,
                                        const Real c,
                                        const Real cos_alpha) noexcept {
    static_assert(std::is_arithmetic_v<Real>);

    Expects(b > 0.);
    Expects(c > 0.);
    // => alpha is smaller 90°
    Expects(cos_alpha > 0.);
    // => alpha is bigger 0°
    Expects(cos_alpha < 1.);

    const Real nom   = (b - c * cos_alpha);
    const Real denom = (b * b + c * c - 2. * b * c * cos_alpha);

    Real ratio = nom / denom;

    /// Note: Because of inaccuracy of floating point operations it is possible
    /// to get abs(ratio) == 1. This is not an error in the implementation
    /// but rather an numerical artifact. To not hit the post-conditions
    /// it is ok to subtract a tiny epsilon.
    /// The problem only occured with 'float' but did not occur with 'double'.
    if (ratio >= 1.) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        ratio = 1. - Real(10.) * std::numeric_limits<Real>::epsilon();
    }
    if (ratio <= -1.) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        ratio = -1. + Real(10.) * std::numeric_limits<Real>::epsilon();
    }

    Ensures(ratio > -1.);
    Ensures(ratio < +1.);

    const Real result = std::acos(ratio);

    Ensures(result > 0.);
    Ensures(result < math::pi<Real>);

    return result;
}

}}  // namespace sens_loc::math

#endif /* end of include guard: TRIANGLES_H_XWPDRVKT */
