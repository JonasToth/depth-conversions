#ifndef PINHOLE_H_I3RZULX9
#define PINHOLE_H_I3RZULX9

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>
#include <utility>

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
struct pinhole {
    int  w  = 0;    ///< width of the image
    int  h  = 0;    ///< height of the image
    Real fx = 0.0;  ///< x-coordinate of focal length
    Real fy = 0.0;  ///< y-corrdiante of focal length
    Real cx = 0.0;  ///< x-coordinate of image center
    Real cy = 0.0;  ///< y-coordinate of image center

    Real p1 = 0.0;  ///< first order tangential distortion coefficient
    Real p2 = 0.0;  ///< second order tangential distortion coefficient

    Real k1 = 0.0;  ///< first order radial distortion coefficient
    Real k2 = 0.0;  ///< second order radial distortion coefficient
    Real k3 = 0.0;  ///< third order radial distortion coefficient

    /// This method calculates the angle of the rays between two pixels.
    /// \param p1,p2 non-negative pixel coordinates smaller \c w and \c h.
    /// \note This uses \p project_to_sphere internally.
    /// \sa project_to_sphere
    /// \returns radians of the angle between the two lightrays.
    template <typename Number = int>
    [[nodiscard]] Real phi(const math::pixel_coord<Number> &p1,
                           const math::pixel_coord<Number> &p2) const noexcept;

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
};

template <typename Real>
template <typename Number>
inline Real pinhole<Real>::phi(const math::pixel_coord<Number> &p1,
                               const math::pixel_coord<Number> &p2) const
    noexcept {
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
math::image_coord<Real>
pinhole<Real>::transform_to_image(const math::pixel_coord<Number> &p) const
    noexcept {
    Expects(fx > 0.);
    Expects(fy > 0.);
    Expects(cx > 0.);
    Expects(cy > 0.);
    Expects(p1 == 0.);
    Expects(p2 == 0.);
    Expects(k1 == 0.);
    Expects(k2 == 0.);
    Expects(k3 == 0.);
    Expects(p.u() >= 0.);
    Expects(p.u() < w);
    Expects(p.v() >= 0.);
    Expects(p.v() < h);
    return math::image_coord<Real>((p.u() - cx) / fx, (p.v() - cy) / fy);
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
