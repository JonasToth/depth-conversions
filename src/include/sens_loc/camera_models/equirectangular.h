#ifndef LASER_SCANNER_H_WSZJIY40
#define LASER_SCANNER_H_WSZJIY40

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/scaling.h>
#include <stdexcept>

namespace sens_loc { namespace camera_models {

namespace detail {
template <typename Real>
Real get_d_phi(int width) noexcept {
    return Real(2.) * math::pi<Real> / Real(width);
}
}  // namespace detail

/// This struct contains the parameters for describing the equirectangular
/// projection of a laser scan.
///
/// The scan is expected to be 360° in horizontal direction and som
/// varying degree in vertical direction.
/// The conversions and projections are mainly conversion from a
/// spherical coordinate-system to a cartesian.
/// \tparam Real precision of the parameters and calculations.
///
/// \note the coordinate convention for spherical coordinates is as follows:
///   - \f$\varphi \in [0, 2 \pi] \mapsto u\f$ note that this wraps around
///   - \f$\theta \in [0, \pi] \mapsto v\f$ - not wrapping!
///   - it is possible to have a different range for \f$\theta\f$ but not for
///     \f$\varphi\f$.
template <typename Real = float>
class equirectangular {
  public:
    /// Default initialize all parameters to zero.
    equirectangular() = default;

    /// Construct a equirectangular image for the full sphere.
    ///
    /// \param width width of the image, this maps to 360° field of vie
    /// horizontally which wraps around
    /// \param height height of the image, this
    /// maps to 180° field of view vertically which does __NOT__ wrap.
    equirectangular(int width, int height) noexcept
        : w(width)
        , h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta(math::pi<Real> / Real(h))
        , theta_min(Real(0.)) {
        Expects(width > 0);
        Expects(height > 0);
        ensure_invariant();
    }

    /// Construct the model with a custom \f$\theta\f$-range. This model does
    /// __NOT__ map to the whole sphere, but is not a cylindrical coordinate
    /// system as well!
    /// \param width,height image dimensions
    /// \param theta_range minimum and maximum angle on the unit-sphere in
    /// vertical direction.
    equirectangular(int width, int height,
                    math::numeric_range<Real> theta_range) noexcept
        : w(width)
        , h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta((theta_range.max - theta_range.min) / Real(h))
        , theta_min(theta_range.min) {
        Expects(width > 0);
        Expects(height > 0);
        Expects(theta_range.min >= 0.);
        Expects(theta_range.max <= math::pi<Real>);
        ensure_invariant();
    }

    /// Construct the model with a minimum angle \p theta_min and an angle
    /// increment \p d_theta.
    ///
    /// \param width,height image dimensions
    /// \param theta_min,d_theta vertical resolution configuration
    /// \throws if the \f$\theta\f$-angle would be out of the range with the
    /// configuration this constructor throws an exception.
    equirectangular(int width, int height, Real theta_min, Real d_theta)
        : w(width)
        , h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta(d_theta)
        , theta_min(theta_min) {
        const Real theta_max = theta_min + height * d_theta;
        if (theta_max > math::pi<Real>)
            throw std::invalid_argument("angle increment too big");
        ensure_invariant();
    }

    /// This method calculates the angle of the rays between two pixels.
    /// \param p1,p2 non-negative pixel coordinates smaller \c w and \c h.
    /// \sa project_to_sphere
    /// \returns radians of the angle between the two lightrays.
    template <typename Number = int>
    [[nodiscard]] Real phi(const math::pixel_coord<Number> &p1,
                           const math::pixel_coord<Number> &p2) const noexcept;

    /// This methods calculates the inverse projection of the equirectangular
    /// model to get the direction of the lightray for the pixel at \p p.
    ///
    /// \param p non-negative pixel coordinates
    /// \post \f$\lVert result \rVert_2 = 1.\f$
    /// \returns normalized vector in camera coordinates - unit sphere
    /// coordinate
    template <typename Number = int>
    [[nodiscard]] math::sphere_coord<Real>
    pixel_to_sphere(const math::pixel_coord<Number> &p) const noexcept;

  private:
    void ensure_invariant() const noexcept {
        Ensures(d_phi > Real(0.));
        Ensures(d_theta > Real(0.));
        Ensures(theta_min >= Real(0.));

        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        Ensures(std::abs(d_phi * w - Real(2.) * math::pi<Real>) < 0.00001);
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        Ensures(theta_min + h * d_theta <= math::pi<Real> + 0.00001);
    }

    int  w;          ///< width of a laser-scan image
    int  h;          ///< height of a laser-scan image.
    Real d_phi;      ///< Angle increment in u-direction.
    Real d_theta;    ///< Angle increment in v-direction.
    Real theta_min;  ///< Smallest angle in v-direction.
};

template <typename Real>
template <typename Number>
inline Real
equirectangular<Real>::phi(const math::pixel_coord<Number> &p1,
                           const math::pixel_coord<Number> &p2) const noexcept {
    Expects(p1.u() >= 0);
    Expects(p1.u() < w);

    Expects(p1.v() >= 0);
    Expects(p1.v() < h);

    Expects(p2.u() >= 0);
    Expects(p2.u() < w);

    Expects(p2.v() >= 0);
    Expects(p2.v() < h);

    const auto s1      = pixel_to_sphere(p1);
    const auto s2      = pixel_to_sphere(p2);
    const auto cos_phi = s1.dot(s2);

    Ensures(cos_phi > -1.);
    Ensures(cos_phi < +1.);

    const auto angle = std::acos(cos_phi);
    return angle;
}

template <typename Real>
template <typename Number>
inline math::sphere_coord<Real>
equirectangular<Real>::pixel_to_sphere(const math::pixel_coord<Number> &p) const
    noexcept {
    const Real phi   = p.u() * d_phi - math::pi<Real>;
    const Real theta = theta_min + (p.v() * d_theta);

    Ensures(phi >= -math::pi<Real>);
    Ensures(phi <= math::pi<Real>);
    Ensures(theta >= 0.);
    Ensures(theta <= math::pi<Real>);

    using std::cos;
    using std::sin;
    math::sphere_coord<Real> s{sin(theta) * cos(phi), sin(theta) * sin(phi),
                               cos(theta)};
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    Ensures(std::abs(s.norm() - Real(1.0)) < 0.000001);
    return s;
}
}}  // namespace sens_loc::camera_models

#endif /* end of include guard: LASER_SCANNER_H_WSZJIY40 */
