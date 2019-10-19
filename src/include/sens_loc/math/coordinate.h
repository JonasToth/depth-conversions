#ifndef COORDINATE_H_Y7FJINQO
#define COORDINATE_H_Y7FJINQO

#include <gsl/gsl>
#include <sens_loc/math/eigen_types.h>
#include <type_traits>

namespace sens_loc { namespace math {

/// This enum allows to discrimate in which coordinate system or frame of
/// reference a vector lives. It is solely for code readability and correctness
/// evaluation.
enum class coordinate_frame {
    world,   ///< 3D, Global, World Coordinate System. \f$(U, V, W)\f$.
    camera,  ///< 3D, Camera coordinate system, but the camera center is the
             ///< frame reference. \f$(X, Y, Z)\f$
    sphere,  ///< 3D, projection from image into space. \f$(X_s, Y_s, Z_s)\f$.
             ///< Equivalent to the direction of a lightray for a pixel.
    image,   ///< 2D, coordinate in an image as floating point. \f$(x, y)\f$
    pixel,   ///< 2D, usually a discrete coordinate system that refers to pixels
             ///< in an image. \f$(u, v)\f$
};

template <coordinate_frame Frame>
struct is_2d
    : std::bool_constant<Frame == coordinate_frame::image ||
                         Frame == coordinate_frame::pixel> {};

template <typename Real>
struct coordinate2d {
    coordinate2d()
        : v(Real(0.), Real(0.)) {}
    coordinate2d(Real x1, Real x2)
        : v(x1, x2) {}

    vec<Real, 2> v;
};

template <typename Real>
struct coordinate3d {
    coordinate3d()
        : v(Real(0.), Real(0.), Real(0.)) {}
    coordinate3d(Real x1, Real x2, Real x3)
        : v(x1, x2, x3) {}

    vec<Real, 3> v;
};

/// This class is a strongly typed coordinate class to ensure the implemented
/// math does not mix up reference frames.
///
/// \tparam Real underlying type of the coordinates
/// \tparam Frame type of reference frame, results in either 2 or 3 dimensional
/// vectors.
template <typename Real, coordinate_frame Frame>
struct coordinate
    : std::conditional_t<is_2d<Frame>::value, coordinate2d<Real>,
                         coordinate3d<Real>> {
    using base_class =
        std::conditional_t<is_2d<Frame>::value, coordinate2d<Real>,
                           coordinate3d<Real>>;
    // Reuse constructors of baseclass
    using base_class::base_class;

    Real  operator[](int i) const noexcept { return base_class::v[i]; }
    Real &operator[](int i) noexcept { return base_class::v[i]; }
};

}}  // namespace sens_loc::math

#endif /* end of include guard: COORDINATE_H_Y7FJINQO */
