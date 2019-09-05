#ifndef DERIVATIVES_H_5CHQ89V7
#define DERIVATIVES_H_5CHQ89V7

#include <gsl/gsl>

namespace sens_loc { namespace math {

/// Calculate the first derivate with the central differential quotient.
/// \param y__1 == y_{i-1}
/// \param y_1  == y_{i+1}
/// \param dx   == dx
template <typename Real>
Real first_derivative_central(Real y__1, Real y_1, Real dx) noexcept {
    Expects(dx > Real(0.));
    return (y_1 - y__1) / (2. * dx);
}

/// Calculate the second derivate with the central differential quotient.
/// \param y__1 == y_{i-1}
/// \param y_0  == y_{i}
/// \param y_1  == y_{i+1}
/// \param dx*dx== dx_squared
template <typename Real>
Real second_derivative_central(Real y__1, Real y_0, Real y_1,
                               Real dx_squared) noexcept {
    Expects(dx_squared > Real(0.));
    return (y_1 + y__1 - 2. * y_0) / (dx_squared);
}
}}  // namespace sens_loc::math

#endif /* end of include guard: DERIVATIVES_H_5CHQ89V7 */
