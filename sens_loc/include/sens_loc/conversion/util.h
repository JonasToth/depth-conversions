#ifndef UTIL_H_QNW3WCZL
#define UTIL_H_QNW3WCZL

#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <type_traits>

namespace sens_loc { namespace conversion {

enum class direction {
    horizontal,  //< calculate the angle from left to right in horizontal
                 //< direction
    vertical,  //< calculate the angle from top to bottom in vertical direction
    diagonal,  //< calculate the bearing angle in main diagnoal direction
               //< (top-left to bottom-right)
    antidiagonal,  //< calculate the bearing angle in the anti-diagnoal
                   //< direction (bottom-left to top-right)
};

namespace detail {

/// Convert the orthografic depth of a pixel into the euclidian distance
/// suggested by the pinhole model.
template <typename Real = float, typename PixelType = ushort>
inline Real
orthografic_to_euclidian(int u, int v, PixelType d,
                         const camera_models::pinhole &intrinsic) noexcept {

    if (d == 0)
        return Real(0.);

    Expects(d > PixelType(0));
    Expects(intrinsic.fx > 0.);
    Expects(intrinsic.fy > 0.);
    Expects(intrinsic.cx > 0.);
    Expects(intrinsic.cy > 0.);
    Expects(intrinsic.k2 == 0.);
    Expects(intrinsic.k1 == 0.);
    Expects(intrinsic.k2 == 0.);
    Expects(intrinsic.k3 == 0.);
    Expects(intrinsic.p1 == 0.);
    Expects(intrinsic.p2 == 0.);

    const Real x        = (u - intrinsic.cx) / intrinsic.fx;
    const Real y        = (v - intrinsic.cy) / intrinsic.fy;
    const Real z_helper = x * x + y * y + 1.;

    const Real euclid_distance = Real(d) * (z_helper) / std::sqrt(z_helper);
    Ensures(euclid_distance >= Real(d));

    return euclid_distance;
}

/// Return the scaling factor for bearing angle conversion.
template <typename Real, typename PixelType>
inline constexpr std::pair<Real, Real> scaling_factor(Real max_angle) {
    const Real min = std::numeric_limits<PixelType>::min();
    const Real max = std::numeric_limits<PixelType>::max();

    return std::make_pair((max - min) / max_angle, -min);
}

template <typename Number>  // requires Number<Number>
inline int get_cv_type() {
    if constexpr (std::is_same<Number, float>::value)
        return CV_32F;
    else if constexpr (std::is_same<Number, double>::value)
        return CV_64F;
    else if constexpr (std::is_same<Number, uchar>::value)
        return CV_8U;
    else if constexpr (std::is_same<Number, schar>::value)
        return CV_8S;
    else if constexpr (std::is_same<Number, ushort>::value)
        return CV_16U;
    else if constexpr (std::is_same<Number, short>::value)
        return CV_16S;
    else
        return -1;
}
}  // namespace detail
}}  // namespace sens_loc::conversion

#endif /* end of include guard: UTIL_H_QNW3WCZL */
