#ifndef COORDINATE_H_Y7FJINQO
#define COORDINATE_H_Y7FJINQO

#include <gsl/gsl>
#include <sens_loc/math/eigen_types.h>
#include <type_traits>

namespace sens_loc { namespace math {

/// This enum allows to discrimate in which coordinate system or frame of
/// reference a vector lives. It is solely for code readability and correctness
/// evaluation.
enum class frame {
    world,   ///< 3D, Global, World Coordinate System. \f$(U, V, W)\f$.
    camera,  ///< 3D, Camera coordinate system, but the camera center is the
             ///< frame reference. \f$(X, Y, Z)\f$
    sphere,  ///< 3D, projection from image into space. \f$(X_s, Y_s, Z_s)\f$.
             ///< Equivalent to the direction of a lightray for a pixel.
    image,   ///< 2D, coordinate in an image as floating point. \f$(x, y)\f$
    pixel,   ///< 2D, usually a discrete coordinate system that refers to pixels
             ///< in an image. \f$(u, v)\f$
};

template <frame Frame>
struct is_2d
    : std::bool_constant<Frame == frame::image || Frame == frame::pixel> {};

template <typename Real>
struct coordinate2d {
    coordinate2d()
        : d(Real(0.), Real(0.)) {}
    coordinate2d(Real x1, Real x2)
        : d(x1, x2) {}

    vec<Real, 2> d;
};

template <typename Real>
struct coordinate3d {
    coordinate3d()
        : d(Real(0.), Real(0.), Real(0.)) {}
    coordinate3d(Real x1, Real x2, Real x3)
        : d(x1, x2, x3) {}

    vec<Real, 3> d;
};

/// This class is a strongly typed coordinate class to ensure the implemented
/// math does not mix up reference frames.
///
/// \tparam Real underlying type of the coordinates
/// \tparam Frame type of reference frame, results in either 2 or 3 dimensional
/// vectors.
template <typename Real, frame Frame>
class coordinate
    : private std::conditional_t<is_2d<Frame>::value, coordinate2d<Real>,
                                 coordinate3d<Real>> {
    static_assert(std::is_arithmetic_v<Real>,
                  "Arithmetic type for Real required!");

    double foo = 42;
    using base_class =
        std::conditional_t<is_2d<Frame>::value, coordinate2d<Real>,
                           coordinate3d<Real>>;

  public:
    // Reuse constructors of baseclass
    using base_class::base_class;

    /// Calculate the euclidean norm for the coordinate.
    /// \note This function is only enabled if the \p Real type is a floating
    /// point number.
    template <typename T = Real,
              typename   = std::enable_if_t<!std::is_integral_v<T>>>
    Real norm() const noexcept {
        return base_class::d.norm();
    }

    /// Return the normalized vector of this coordinate.
    /// \note This function is only enabled if the \p Real type is a floating
    /// point number and the vector is in 3 dimensional space.
    template <typename T = Real, typename U = frame,
              typename = std::enable_if_t<!std::is_integral_v<T> &&
                                          !is_2d<Frame>::value>>
    coordinate<Real, Frame> normalized() const noexcept {
        const auto res = base_class::d.normalized();
        return {res[0], res[1], res[2]};
    }

    /// Calculate the dot product with \p other.
    Real dot(const coordinate<Real, Frame> &other) const noexcept {
        return base_class::d.dot(other.d);
    }
    /// Calculate the cross product with \p other.
    template <typename T = frame, typename = std::enable_if_t<!(
                                      Frame == T::pixel || Frame == T::image)>>
    coordinate<Real, Frame> cross(const coordinate<Real, Frame> &other) const
        noexcept {
        const auto res = base_class::d.cross(other.d);
        return {res[0], res[1], res[2]};
    }

    // clang-format off
    template <typename T = frame,
              typename   = std::enable_if_t<Frame == T::pixel>>
    Real u() const noexcept { return base_class::d[0]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::pixel>>
    Real v() const noexcept { return base_class::d[1]; }


    template <typename T = frame,
              typename   = typename std::enable_if_t<Frame == T::image>>
    Real x() const noexcept { return base_class::d[0]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::image>>
    Real y() const noexcept { return base_class::d[1]; }


    template <typename T = frame,
              typename   = typename std::enable_if_t<Frame == T::sphere>>
    Real Xs() const noexcept { return base_class::d[0]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::sphere>>
    Real Ys() const noexcept { return base_class::d[1]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::sphere>>
    Real Zs() const noexcept { return base_class::d[2]; }


    template <typename T = frame,
              typename   = typename std::enable_if_t<Frame == T::camera>>
    Real X() const noexcept { return base_class::d[0]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::camera>>
    Real Y() const noexcept { return base_class::d[1]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::camera>>
    Real Z() const noexcept { return base_class::d[2]; }


    template <typename T = frame,
              typename   = typename std::enable_if_t<Frame == T::world>>
    Real U() const noexcept { return base_class::d[0]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::world>>
    Real V() const noexcept { return base_class::d[1]; }
    template <typename T = frame,
              typename   = typename std::enable_if<Frame == T::world>>
    Real W() const noexcept { return base_class::d[2]; }
    // clang-format on

    coordinate<Real, Frame> operator-(const coordinate<Real, Frame> &p) const
        noexcept {
        coordinate<Real, Frame> res;
        auto                    difference = base_class::d - p.d;

        res.d = difference;
        return res;
    }
};

template <typename Real = float>
using pixel_coord = coordinate<Real, frame::pixel>;
template <typename Real = float>
using image_coord = coordinate<Real, frame::image>;
template <typename Real = float>
using sphere_coord = coordinate<Real, frame::sphere>;
template <typename Real = float>
using camera_coord = coordinate<Real, frame::camera>;
template <typename Real = float>
using world_coord = coordinate<Real, frame::world>;

}}  // namespace sens_loc::math

#endif /* end of include guard: COORDINATE_H_Y7FJINQO */
