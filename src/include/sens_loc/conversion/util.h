#ifndef UTIL_H_QNW3WCZL
#define UTIL_H_QNW3WCZL

#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/coordinate.h>
#include <type_traits>

namespace sens_loc {

/// All conversion functions for depth images are implemented in this namespace.
/// It is the core of the whole project.
namespace conversion {

/// Helper type for bearing-angle calculations to specify the desired
/// neighbourhood relationship.
enum class direction {
    horizontal,    ///< Calculate the angle from left to right in horizontal
                   ///< direction.
    vertical,      ///< Calculate the angle from top to bottom in vertical
                   ///< direction.
    diagonal,      ///< Calculate the bearing angle in main diagnoal direction
                   ///< (top-left to bottom-right).
    antidiagonal,  ///< Calculate the bearing angle in the anti-diagnoal
                   ///< direction (bottom-left to top-right).
};

namespace detail {

/// Convert the orthografic depth of a pixel into the euclidian distance
/// suggested by the pinhole model.
template <typename Real = float, typename PixelType = ushort>
inline Real orthografic_to_euclidian(
    math::pixel_coord<int> p, PixelType d,
    const camera_models::pinhole<Real> &intrinsic) noexcept {
    if (d == 0)
        return Real(0.);

    Expects(d > PixelType(0));

    const math::image_coord<Real> p_i      = intrinsic.transform_to_image(p);
    const Real                    x        = p_i.x();
    const Real                    y        = p_i.y();
    const Real                    z_helper = x * x + y * y + 1.;

    const Real euclid_distance = Real(d) * (z_helper) / std::sqrt(z_helper);
    Ensures(euclid_distance >= Real(d));

    return euclid_distance;
}

/// Return the scaling factor for bearing angle conversion.
template <typename Real, typename PixelType>
inline constexpr std::pair<Real, Real> scaling_factor(Real max_angle) {
    const Real min = std::numeric_limits<PixelType>::min();
    const Real max = std::numeric_limits<PixelType>::max();

    return std::make_pair((max - min) / max_angle, min);
}
}  // namespace detail
}  // namespace conversion
}  // namespace sens_loc

#endif /* end of include guard: UTIL_H_QNW3WCZL */
