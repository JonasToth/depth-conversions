#ifndef CURVATURE_H_LSXONAFA
#define CURVATURE_H_LSXONAFA

#include <cmath>
#include <gsl/gsl>

namespace sens_loc { namespace math {

/// This function calculates the gaussian curvature for a function given its
/// derivatives.
/// https://de.wikipedia.org/wiki/Gau%C3%9Fsche_Kr%C3%BCmmung
template <typename Real>
inline Real gaussian_curvature(Real f_u, Real f_v, Real f_uu, Real f_vv,
                               Real f_uv) noexcept {
    return (f_uu * f_vv - f_uv * f_uv) / (Real(1.) + f_u * f_u + f_v * f_v);
}

/// This function calculates the mean curvature for a function given its
/// derivatives.
/// https://de.wikipedia.org/wiki/Mittlere_Kr%C3%BCmmung
template <typename Real>
inline Real mean_curvature(Real f_u, Real f_v, Real f_uu, Real f_vv,
                           Real f_uv) noexcept {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    return ((Real(1.) + f_v * f_v) * f_uu - Real(2.) * f_u * f_v * f_uv +
            (Real(1.) + f_u * f_u) * f_vv) /
           // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
           std::pow(Real(2.) * std::sqrt(Real(1.) + f_u * f_u + f_v * f_v),
                    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
                    Real(3.));
}
}}  // namespace sens_loc::math

#endif /* end of include guard: CURVATURE_H_LSXONAFA */
