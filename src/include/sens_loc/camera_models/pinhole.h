#ifndef PINHOLE_H_I3RZULX9
#define PINHOLE_H_I3RZULX9

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>

namespace sens_loc {

/// This namespace contains all camera and projection models and implements
/// them as necessary.
namespace camera_models {

/// This struct contains all parameters for the general pinhole camera model.
/// For more information you can check the
/// <a href="https://docs.opencv.org/master/d9/d0c/group__calib3d.html"
/// target="_blank">OpenCV documentation (Section 'Detailed Description')</a>.
///
/// The struct itself provides some helper functions useful in the code but is
/// not smart itself.
/// \warning Each value can be modified by everyone, as its just as list of
/// numbers. Use \c const in your code to prevent bugs!
template <typename Real = float>
class pinhole {
  public:
    using real_type = Real;

    constexpr pinhole() = default;

    /// Initialize the pinhole model with it's essential parameters.
    /// \param w,h image dimensions
    /// \param fx,fy focal point
    /// \param cx,cy image center
    constexpr pinhole(int w, int h, Real fx, Real fy, Real cx, Real cy) noexcept
        : _w(w)
        , _h(h)
        , _fx(fx)
        , _fy(fy)
        , _cx(cx)
        , _cy(cy) {
        Expects(w > 0);
        Expects(h > 0);
        Expects(_fx > Real(0.));
        Expects(_fy > Real(0.));
        Expects(_cx > Real(0.));
        Expects(_cy > Real(0.));
    }

    /// Return the width of the image corresponding to this calibration.
    [[nodiscard]] int w() const noexcept { return _w; }
    /// Return the height of the image corresponding to this calibration.
    [[nodiscard]] int h() const noexcept { return _h; }
    /// Return the x-component of the focal point.
    [[nodiscard]] Real fx() const noexcept { return _fx; }
    /// Return the y-component of the focal point.
    [[nodiscard]] Real fy() const noexcept { return _fy; }
    /// Return the x-component of the image center.
    [[nodiscard]] Real cx() const noexcept { return _cx; }
    /// Return the y-component of the image center.
    [[nodiscard]] Real cy() const noexcept { return _cy; }

    template <typename Number = int>
    [[nodiscard]] math::image_coord<Real>
    transform_to_image(const math::pixel_coord<Number> &p) const noexcept;

    /// This methods calculates the inverse projection of the camera model
    /// to get the direction of the lightray for the pixel at \p p.
    ///
    /// \param p non-negative pixel coordinates
    /// \warning This does not respect a distortion model at all!
    /// \pre \p fx > 0
    /// \pre \p fy > 0
    /// \pre \p cx > 0
    /// \pre \p cy > 0
    /// \pre \p p1 == \p p2 == \p k1 == \p k2 == \p k3 == 0!!
    /// \post \f$\lVert result \rVert_2 = 1.\f$
    /// \returns normalized vector in camera coordinates
    template <typename Number = int>
    [[nodiscard]] math::sphere_coord<Real>
    pixel_to_sphere(const math::pixel_coord<Number> &p) const noexcept;

    /// The same as \p project_to_sphere but the coordinate is already
    /// in the image frame of reference.
    /// \sa pinhole::project_to_sphere
    [[nodiscard]] math::sphere_coord<Real>
    image_to_sphere(const math::image_coord<Real> &p) const noexcept;

  private:
    int  _w  = 0;    ///< width of the image
    int  _h  = 0;    ///< height of the image
    Real _fx = 0.0;  ///< x-coordinate of focal length
    Real _fy = 0.0;  ///< y-corrdiante of focal length
    Real _cx = 0.0;  ///< x-coordinate of image center
    Real _cy = 0.0;  ///< y-coordinate of image center

    Real p1 = 0.0;  ///< first order tangential distortion coefficient
    Real p2 = 0.0;  ///< second order tangential distortion coefficient

    Real k1 = 0.0;  ///< first order radial distortion coefficient
    Real k2 = 0.0;  ///< second order radial distortion coefficient
    Real k3 = 0.0;  ///< third order radial distortion coefficient
};

template <typename Real>
template <typename Number>
math::image_coord<Real>
pinhole<Real>::transform_to_image(const math::pixel_coord<Number> &p) const
    noexcept {
    Expects(_fx > 0.);
    Expects(_fy > 0.);
    Expects(_cx > 0.);
    Expects(_cy > 0.);
    Expects(p1 == 0.);
    Expects(p2 == 0.);
    Expects(k1 == 0.);
    Expects(k2 == 0.);
    Expects(k3 == 0.);
    Expects(p.u() >= 0.);
    Expects(p.u() < w());
    Expects(p.v() >= 0.);
    Expects(p.v() < h());
    return math::image_coord<Real>((p.u() - _cx) / _fx, (p.v() - _cy) / _fy);
}

template <typename Real>
template <typename Number>
inline math::sphere_coord<Real>
pinhole<Real>::pixel_to_sphere(const math::pixel_coord<Number> &p) const
    noexcept {
    return image_to_sphere(transform_to_image(p));
}

template <typename Real>
inline math::sphere_coord<Real>
pinhole<Real>::image_to_sphere(const math::image_coord<Real> &p) const
    noexcept {
    const double x = p.x();
    const double y = p.y();
    const double z = 1.;

    const double factor = std::sqrt(1. + x * x + y * y) / (1. + x * x + y * y);
    math::sphere_coord<Real> res(factor * x, factor * y, factor * z);

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    Ensures(std::abs(res.norm() - 1.) < 0.000001);

    return res;
}
}  // namespace camera_models
}  // namespace sens_loc


#endif /* end of include guard: PINHOLE_H_I3RZULX9 */
