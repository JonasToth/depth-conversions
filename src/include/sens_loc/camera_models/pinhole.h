#ifndef PINHOLE_H_I3RZULX9
#define PINHOLE_H_I3RZULX9

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/camera_models/concepts.h>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>
#include <type_traits>

namespace sens_loc {

/// This namespace contains all camera and projection models and implements
/// them as necessary.
namespace camera_models {

/// This struct contains all parameters for the general pinhole camera model.
/// For more information you can check the
/// <a href="https://docs.opencv.org/master/d9/d0c/group__calib3d.html"
/// target="_blank">OpenCV documentation (Section 'Detailed Description')</a>.
///
/// \tparam Real floating point type that determines the precision of the
/// calculations.
/// \sa is_intrinsic_v
template <typename Real = float>
class pinhole {
  public:
    static_assert(std::is_floating_point_v<Real>);
    using real_type = Real;

    pinhole() = default;

    /// Initialize the pinhole model with it's essential parameters.
    /// \param w,h image dimensions
    /// \param fx,fy focal point
    /// \param cx,cy image center
    pinhole(int w, int h, Real fx, Real fy, Real cx, Real cy) noexcept
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
    transform_to_image(const math::pixel_coord<Number>& p) const noexcept;

    /// This methods calculates the inverse projection of the camera model
    /// to get the direction of the lightray for the pixel at \p p.
    ///
    /// \tparam _Real either integer or floating point value for pixel or
    /// subpixel precision
    /// \param p non-negative pixel coordinates
    /// \warning This does not respect a distortion model at all!
    /// \pre \p fx > 0
    /// \pre \p fy > 0
    /// \pre \p cx > 0
    /// \pre \p cy > 0
    /// \pre \p p1 == \p p2 == \p k1 == \p k2 == \p k3 == 0!!
    /// \post \f$\lVert result \rVert_2 = 1.\f$
    /// \returns normalized vector in camera coordinates
    /// \note if \p _Real is an integer-type the value is itself backprojected,
    /// which usually means the bottom left corner of the pixel and __NOT__
    /// its center!
    template <typename _Real = int>
    [[nodiscard]] math::sphere_coord<Real>
    pixel_to_sphere(const math::pixel_coord<_Real>& p) const noexcept;

    /// The same as \p project_to_sphere but the coordinate is already
    /// in the image frame of reference.
    /// \sa pinhole::project_to_sphere
    [[nodiscard]] math::sphere_coord<Real>
    image_to_sphere(const math::image_coord<Real>& p) const noexcept;

    /// Project points in camera coordinates to pixel coordinates.
    /// \note if the point can not be projected (\c p.z() == 0) the pixel
    /// coordinate {-1, -1} is returned.
    /// \sa pinhole::project_to_sphere
    template <typename _Real = Real>
    [[nodiscard]] math::pixel_coord<_Real>
    camera_to_pixel(const math::camera_coord<Real>& p) const noexcept;

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
template <typename _Real>
math::image_coord<Real>
pinhole<Real>::transform_to_image(const math::pixel_coord<_Real>& p) const
    noexcept {
    static_assert(std::is_arithmetic_v<_Real>);
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
template <typename _Real>
inline math::sphere_coord<Real>
pinhole<Real>::pixel_to_sphere(const math::pixel_coord<_Real>& p) const
    noexcept {
    static_assert(std::is_arithmetic_v<_Real>);
    return image_to_sphere(transform_to_image(p));
}

template <typename Real>
inline math::sphere_coord<Real>
pinhole<Real>::image_to_sphere(const math::image_coord<Real>& p) const
    noexcept {
    const double x = p.x();
    const double y = p.y();
    const double z = 1.;

    const double cse    = 1. + x * x + y * y;
    const double factor = 1. / std::sqrt(cse);

    using gsl::narrow_cast;
    math::sphere_coord<Real> res(narrow_cast<Real>(factor * x),
                                 narrow_cast<Real>(factor * y),
                                 narrow_cast<Real>(factor * z));

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    Ensures(std::abs(res.norm() - 1.) < 0.000001);

    return res;
}
template <typename Real>
template <typename _Real>
math::pixel_coord<_Real>
pinhole<Real>::camera_to_pixel(const math::camera_coord<Real>& p) const
    noexcept {
    static_assert(std::is_arithmetic_v<_Real>);
    Expects(fx() > 0.);
    Expects(fy() > 0.);
    Expects(cx() > 0.);
    Expects(cy() > 0.);
    Expects(p1 == 0.);
    Expects(p2 == 0.);
    Expects(k1 == 0.);
    Expects(k2 == 0.);
    Expects(k3 == 0.);

    if (p.Z() == Real(0.0))
        return {_Real(-1), _Real(-1)};

    const math::image_coord<Real> i{p.X() / p.Z(), p.Y() / p.Z()};
    const auto u = gsl::narrow_cast<_Real>(fx() * i.x() + cx());
    const auto v = gsl::narrow_cast<_Real>(fy() * i.y() + cy());

    if (u < _Real(0.0) || u > gsl::narrow_cast<Real>(w()) || v < _Real(0.0) ||
        v > gsl::narrow_cast<Real>(h()))
        return {_Real(-1), _Real(-1)};

    return {u, v};
}
}  // namespace camera_models
}  // namespace sens_loc


#endif /* end of include guard: PINHOLE_H_I3RZULX9 */
