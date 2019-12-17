#ifndef CURVATURE_H_LSXONAFA
#define CURVATURE_H_LSXONAFA

#include <cmath>
#include <gsl/gsl>
#include <type_traits>

namespace sens_loc::math {

/// This function calculates the gaussian curvature for a function given its
/// derivatives.
/// \tparam Real precision of the calculation
/// \param f_u,f_v,f_uu,f_vv,f_uv partial derivatives of the depth values
/// \returns gaussian curvature for these derivatives
/// \sa conversion::depth_to_gaussian_curvature
/// \note check https://en.wikipedia.org/wiki/Gaussian_curvature for more info
template <typename Real>
inline Real gaussian_curvature(
    Real f_u, Real f_v, Real f_uu, Real f_vv, Real f_uv) noexcept {
    static_assert(std::is_floating_point_v<Real>);

    return (f_uu * f_vv - f_uv * f_uv) / (Real(1.) + f_u * f_u + f_v * f_v);
}

/// This function calculates the mean curvature for a function given its
/// derivatives.
/// \tparam Real precicions of the calculation
/// \param f_u,f_v,f_uu,f_vv,f_uv partial derivatives of the depth values
/// \returns the mean curvature for these derivatives
/// \sa conversion::depth_to_mean_curvature
/// \note check https://en.wikipedia.org/wiki/Mean_curvature for more info
template <typename Real>
inline Real
mean_curvature(Real f_u, Real f_v, Real f_uu, Real f_vv, Real f_uv) noexcept {
    static_assert(std::is_floating_point_v<Real>);

    using std::pow;
    using std::sqrt;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    return ((Real(1.) + f_v * f_v) * f_uu - Real(2.) * f_u * f_v * f_uv +
            (Real(1.) + f_u * f_u) * f_vv) /
           // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
           pow(Real(2.) * sqrt(Real(1.) + f_u * f_u + f_v * f_v), Real(3.));
}
}  // namespace sens_loc::math

#endif /* end of include guard: CURVATURE_H_LSXONAFA */
