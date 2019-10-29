#ifndef ANGLE_CONVERSION_H_1QTGVG9X
#define ANGLE_CONVERSION_H_1QTGVG9X

#include <gsl/gsl>
#include <sens_loc/math/constants.h>
#include <type_traits>

namespace sens_loc {

/// Implement necessary math functionality in this namespace, which is mostly
/// used by conversions.
namespace math {

/// This function converts degree into radians
/// \tparam T precision of the type (e.g. \c float or \c double)
/// \param degree angle in degree
/// \returns the same angle in radians
/// \sa rad_to_deg
/// \pre \f$0. <= degree <= 360.\f$
/// \post \f$0. <= result <= 2\pi\f$
/// \post \c rad_to_deg(result) == \p degree
template <typename T>  // requires(Float(T))
constexpr inline T deg_to_rad(T degree) {
    static_assert(std::is_floating_point_v<T>);

    Expects(degree <= T(360.));
    Expects(degree >= T(0.));
    const T radians = degree / T(180.) * pi<T>;
    Ensures(radians >= T(0.));
    Ensures(radians <= T(2. * pi<T>));

    return radians;
}

/// This function converts radians into degrees
/// \tparam T precision of the type (e.g. \c float or \c double)
/// \param radians angle in radians
/// \returns the same angle in degree
/// \sa deg_to_rad
/// \pre \f$0. <= radians <= 2\pi\f$
/// \post \f$0. <= result <= 360.\f$
/// \post \c deg_to_rad(result) == \p radians
template <typename T>  // requires(Float(T))
constexpr inline T rad_to_deg(T radians) {
    static_assert(std::is_floating_point_v<T>);

    Expects(radians >= T(0.));
    Expects(radians <= T(2. * pi<T>));
    const T degree = radians * T(180.) / pi<T>;
    Ensures(degree <= T(360.));
    Ensures(degree >= T(0.));

    return degree;
}
}}  // namespace sens_loc::math

#endif /* end of include guard: ANGLE_CONVERSION_H_1QTGVG9X */
