#ifndef DERIVATIVES_H_5CHQ89V7
#define DERIVATIVES_H_5CHQ89V7

#include <gsl/gsl>
#include <tuple>

namespace sens_loc { namespace math {

/// Calculate the first derivate with the central differential quotient.
/// \param y__1 == y_{i-1}
/// \param y_1  == y_{i+1}
/// \param dx   == 2. * dx
template <typename Real>
inline Real first_derivative_central(Real y__1, Real y_1, Real dx) noexcept {
    Expects(dx > Real(0.));
    return (y_1 - y__1) / dx;
}

/// Calculate the second derivate with the central differential quotient.
/// \param y__1 == y_{i-1}
/// \param y_0  == y_{i}
/// \param y_1  == y_{i+1}
/// \param dx*dx== dx_squared
template <typename Real>
inline Real second_derivative_central(Real y__1, Real y_0, Real y_1,
                               Real dx_squared) noexcept {
    Expects(dx_squared > Real(0.));
    return (y_1 + y__1 - 2. * y_0) / (dx_squared);
}

/// Calculate the derivatives for a surface patch.
///
/// Index convention:
/// __1 == _{-1}
/// __0 == _{0}
/// _1  == _{1}
///
/// Angle convention:
/// phi -> u direction
/// theta -> v direction
/// The delta is the full angle from the outer points.
///
/// \returns (f_u, f_v, f_uu, f_vv, f_uv)
// clang-format off
template <typename Real = float>
inline std::tuple<Real, Real, Real, Real, Real>
derivatives(Real d__1__1, Real d__1__0, Real d__1_1,
            Real d__0__1, Real d__0__0, Real d__0_1,
            Real d_1__1, Real d_1__0, Real d_1_1,
            Real d_phi, Real d_theta, Real d_phi_theta) noexcept {
    Expects(d_phi > 0.);
    Expects(d_theta > 0.);
    Expects(d_phi_theta > 0.);

    // clang-format on
    const Real f_u  = math::first_derivative_central(d__0__1, d__0_1, d_phi);
    const Real f_v  = math::first_derivative_central(d__1__0, d_1__0, d_theta);
    const Real f_uu = math::second_derivative_central(d__0__1, d__0__0, d__0_1,
                                                      d_phi * d_phi);
    const Real f_vv = math::second_derivative_central(d__1__0, d__0__0, d_1__0,
                                                      d_theta * d_theta);
    const Real f_uv = math::second_derivative_central(
        d__1__0, d__0__0, d_1__0, d_phi_theta * d_phi_theta);

    return std::make_tuple(f_u, f_v, f_uu, f_vv, f_uv);
};
}}  // namespace sens_loc::math

#endif /* end of include guard: DERIVATIVES_H_5CHQ89V7 */
