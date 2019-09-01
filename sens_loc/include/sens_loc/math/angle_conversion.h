#ifndef ANGLE_CONVERSION_H_1QTGVG9X
#define ANGLE_CONVERSION_H_1QTGVG9X

#include <gsl/gsl>
#include <sens_loc/math/constants.h>

namespace sens_loc { namespace math {
template <typename T>  // requires(Float(T))
T deg_to_rad(T degree) {
    Expects(degree <= T(360.));
    Expects(degree >= T(0.));
    const T radians = degree / T(180.) * pi<T>;
    Ensures(radians >= T(0.));
    Ensures(radians <= T(2. * pi<T>));

    return radians;
}
template <typename T>  // requires(Float(T))
T rad_to_deg(T radians) {
    Expects(radians >= T(0.));
    Expects(radians <= T(2. * pi<T>));
    const T degree = radians * T(180.) / pi<T>;
    Ensures(degree <= T(360.));
    Ensures(degree >= T(0.));

    return degree;
}
}}  // namespace sens_loc::math

#endif /* end of include guard: ANGLE_CONVERSION_H_1QTGVG9X */
