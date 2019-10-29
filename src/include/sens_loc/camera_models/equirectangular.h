#ifndef LASER_SCANNER_H_WSZJIY40
#define LASER_SCANNER_H_WSZJIY40

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/camera_models/concepts.h>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/scaling.h>
#include <stdexcept>
#include <type_traits>

namespace sens_loc { namespace camera_models {

namespace detail {
template <typename Real>
Real get_d_phi(int width) noexcept {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
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
///
/// \tparam Real floating point type that determines the precision of the
/// calculations.
/// \sa is_intrinsic_v
template <typename Real = float>
class equirectangular {
  public:
    static_assert(std::is_floating_point_v<Real>);
    using real_type = Real;

    /// Default initialize all parameters to zero.
    equirectangular() = default;

    /// Construct a equirectangular image for the full sphere.
    ///
    /// \param width width of the image, this maps to 360° field of vie
    /// horizontally which wraps around
    /// \param height height of the image, this
    /// maps to 180° field of view vertically which does __NOT__ wrap
    equirectangular(int width, int height) noexcept
        : _w(width)
        , _h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta(math::pi<Real> / Real(height))
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
        : _w(width)
        , _h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta((theta_range.max - theta_range.min) / Real(height))
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
        : _w(width)
        , _h(height)
        , d_phi(detail::get_d_phi<Real>(width))
        , d_theta(d_theta)
        , theta_min(theta_min) {
        const Real theta_max = theta_min + height * d_theta;
        if (theta_max > math::pi<Real>)
            throw std::invalid_argument("angle increment too big");
        ensure_invariant();
    }

    /// Return the width of the image corresponding to this intrinsic.
    [[nodiscard]] int w() const noexcept { return _w; }
    /// Return the height of the image corresponding to this intrinsic.
    [[nodiscard]] int h() const noexcept { return _h; }

    /// This methods calculates the inverse projection of the equirectangular
    /// model to get the direction of the lightray for the pixel at \p p.
    ///
    /// \tparam _Real either integer or floating point value for pixel or
    /// subpixel precision
    /// \param p non-negative pixel coordinates
    /// \post \f$\lVert result \rVert_2 = 1.\f$
    /// \returns normalized vector in camera coordinates - unit sphere
    /// coordinate
    /// \note if \p _Real is an integer-type the value is itself backprojected,
    /// which usually means the bottom left corner of the pixel and __NOT__
    /// its center!
    template <typename _Real = int>
    [[nodiscard]] math::sphere_coord<Real>
    pixel_to_sphere(const math::pixel_coord<_Real> &p) const noexcept;

  private:
    void ensure_invariant() const noexcept {
        Ensures(d_phi > Real(0.));
        Ensures(d_theta > Real(0.));
        Ensures(theta_min >= Real(0.));

        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        Ensures(std::abs(d_phi * _w - Real(2.) * math::pi<Real>) < 0.00001);
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        Ensures(theta_min + _h * d_theta <= math::pi<Real> + 0.00001);
    }

    int  _w        = 0;   ///< width of a laser-scan image
    int  _h        = 0;   ///< height of a laser-scan image.
    Real d_phi     = 0.;  ///< Angle increment in u-direction.
    Real d_theta   = 0.;  ///< Angle increment in v-direction.
    Real theta_min = 0.;  ///< Smallest angle in v-direction.
};

template <typename Real>
template <typename _Real>
inline math::sphere_coord<Real>
equirectangular<Real>::pixel_to_sphere(const math::pixel_coord<_Real> &p) const
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
