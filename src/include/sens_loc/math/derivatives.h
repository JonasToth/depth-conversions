#ifndef DERIVATIVES_H_5CHQ89V7
#define DERIVATIVES_H_5CHQ89V7

#include <gsl/gsl>
#include <tuple>
#include <type_traits>

namespace sens_loc::math {

/// Calculate the first derivate with the central differential quotient.
/// \tparam Real precision of the calculation
/// \param y__1 \f$y_{i-1}\f$
/// \param y_1 \f$y_{i+1}\f$
/// \param dx \f$2. * dx\f$
/// \returns first derivative at this point of order \f$\mathcal{O}(dx^2)\f$
template <typename Real>
inline Real first_derivative_central(Real y__1, Real y_1, Real dx) noexcept {
    static_assert(std::is_floating_point_v<Real>);

    Expects(dx > Real(0.));
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    return (y_1 - y__1) / (Real(2.) * dx);
}

/// Calculate the second derivate with the central differential quotient.
/// \tparam Real precision of the calculation
/// \param y__1 \f$y_{i-1}\f$
/// \param y_0 \f$y_{i}\f$
/// \param y_1 \f$y_{i+1}\f$
/// \param dx \f$dx\f$
/// \returns second derivative at this point of order \f$\mathcal{O}(dx^2)\f$
template <typename Real>
inline Real
second_derivative_central(Real y__1, Real y_0, Real y_1, Real dx) noexcept {
    static_assert(std::is_floating_point_v<Real>);

    Expects(dx > Real(0.));
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    return (y_1 + y__1 - Real(2.) * y_0) / (dx * dx);
}

/// Calculate the derivatives for a surface patch.
///
/// Index convention:
/// \f$d\_\_1 == d_{-1}\f$
/// \f$d\_\_0 == d_{0}\f$
/// \f$d\_1   == d_{1}\f$
///
/// Angle convention:
/// \f$\varphi\f$ -> u direction
/// \f$\theta\f$ -> v direction
///
/// \tparam Real precision of the calculation
/// \param d__1__1,d__1__0,d__1_1 neighbours "above" central pixel
/// \param d__0__1,d__0__0,d__0_1 same row as the central pixel
/// \param d_1__1,d_1__0,d_1_1 row "after" the central pixel
/// \param d_phi angle between rays in x direction \f$(u - 1, u + 1)\f$
/// \param d_theta angle between rays in y direction \f$(v - 1, v + 1)\f$
/// \param d_phi_theta angle between rays in diagonal direction
/// \f$(u - 1, v + 1)\f$
/// \returns partial derivatives \f$(f_u, f_v, f_uu, f_vv, f_uv)\f$
/// \pre the depth values shuold be positive, as they encode depth values
/// \pre \p / d_phi, \p d_theta, \p d_phi_theta are all positive angles
// clang-format off
template <typename Real = float>
inline std::tuple<Real, Real, Real, Real, Real>
derivatives(Real d__1__1, Real d__1__0, Real d__1_1,
            Real d__0__1, Real d__0__0, Real d__0_1,
            Real d_1__1, Real d_1__0, Real d_1_1,
            Real d_phi, Real d_theta, Real d_phi_theta) noexcept {
    static_assert(std::is_floating_point_v<Real>);

    Expects(d_phi > 0.);
    Expects(d_theta > 0.);
    Expects(d_phi_theta > 0.);

    (void)d__1__1;
    (void)d__1_1;
    (void)d_1__1;
    (void)d_1_1;

    // clang-format on
    const Real f_u = math::first_derivative_central(d__0__1, d__0_1, d_phi);
    const Real f_v = math::first_derivative_central(d__1__0, d_1__0, d_theta);
    const Real f_uu =
        math::second_derivative_central(d__0__1, d__0__0, d__0_1, d_phi);
    const Real f_vv =
        math::second_derivative_central(d__1__0, d__0__0, d_1__0, d_theta);
    const Real f_uv =
        math::second_derivative_central(d__1__0, d__0__0, d_1__0, d_phi_theta);

    return std::make_tuple(f_u, f_v, f_uu, f_vv, f_uv);
};
// clang-format on
}  // namespace sens_loc::math

#endif /* end of include guard: DERIVATIVES_H_5CHQ89V7 */
