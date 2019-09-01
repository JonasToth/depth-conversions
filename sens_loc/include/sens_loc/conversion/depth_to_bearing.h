#ifndef DEPTH_TO_BEARING_H_ZXFA9HGG
#define DEPTH_TO_BEARING_H_ZXFA9HGG

#include <cmath>
#include <gsl/gsl>
#include <iostream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/constants.h>
#include <stdexcept>
#include <type_traits>

namespace sens_loc { namespace conversion {

enum class bearing_direction {
    horizontal,  //< calculate the angle from left to right in horizontal
                 //< direction
    vertical,  //< calculate the angle from top to bottom in vertical direction
    diagonal,  //< calculate the bearing angle in main diagnoal direction
               //< (top-left to bottom-right)
    antidiagonal,  //< calculate the bearing angle in the anti-diagnoal
                   //< direction (bottom-left to top-right)
};

/// Convert the image \param depth_image to an bearing angle image.
/// This function returns a new image with the same dimension as
/// \param depth_image but each pixel value is the corresponding bearing angle.
///
/// \Expects \param depth_image to have 1 channel
template <bearing_direction Direction>
cv::Mat
depth_to_bearing(const cv::Mat &                          depth_image,
                 const camera_models::pinhole_parameters &intrinsic) noexcept;


namespace detail {
/// This function calculates the bearing angle between two neighbouring range
/// measurements.
/// \param phi is an angle in raidans. It is the delta between two light rays
/// that and symbolizes the resolution of the sensor. Required to be < 90°.
/// \param cos_phi == std::cos(phi) as an optimization.
/// \param d_i reference depth and > 0
/// \param d_j prior depth and > 0
///
/// The bearing angle is angle between the ray to 'd_i' and the connecting line
/// from 'd_i' to 'd_j'.
/// It is bigger 0° and smaller 180° due to triangle constraints.
template <typename Real>  // requires(Float(Real))
Real ba_formula(Real d_i, Real d_j, Real cos_phi, Real phi) noexcept {
    // 'cos(phi) > 0.' because 'phi < 90°' as reasonable expectation.
    // The formulas only hold as long 'phi' is not the biggest angle in the
    // triangle.
    Expects(cos_phi > 0.);
    // 'cos(phi) < 1.0' because 'phi > 0' otherwise no resolution for scanning
    Expects(cos_phi < 1.);
    Expects(std::abs(std::cos(phi) - cos_phi) < 0.0001);

    // Depth-values need to be non-negative.
    Expects(d_i > 0.);
    Expects(d_j > 0.);

    // Depending on the result tests different equations need to be used
    // for the calculation of the bearing angle.
    // The reason is, that the bearing angle is either sharp, rectangular or
    // blunt. The trigonometric relationships do change for these cases.
    const Real ratio_test = d_i - d_j * cos_phi;

    // The bearing angle is rectangular.
    // 'if (ratio_test == 0.)' is analytical correct, but discretization allows
    // to save some CPU cycles and short-circuit the calculation here.
    if (std::abs(ratio_test) <= 0.0001)
        return math::pi<Real> / 2.;

    const Real radix = (d_i * d_i) + (d_j * d_j) - (2. * d_i * d_j * cos_phi);

    // This means the bearing angle is greater 90°.
    if (ratio_test < 0.) {
        Ensures(radix > 0.);
        const Real bearing = std::acos(ratio_test / std::sqrt(radix));
        // Angle is bigger 90° and smaller 180°.
        Ensures(bearing > math::pi<Real> / 2.);
        Ensures(bearing < math::pi<Real>);
        return bearing;
    }

    // The bearing angle is smaller 90°, because thats the only option left.
    Ensures(ratio_test > 0.);

    const Real cos_bearing = (d_j - d_i * cos_phi) / radix;
    Ensures(cos_bearing > -1.);
    Ensures(cos_bearing < +1.);
    const Real bearing = math::pi<Real> - phi - std::acos(cos_bearing);

    Ensures(bearing <= math::pi<Real> / 2.);
    Ensures(bearing > 0.);
    return bearing;
}

/// Return the scaling factor for bearing angle conversion.
template <typename Real, typename PixelType>
constexpr std::pair<Real, Real> scaling_factor() {
    const Real min       = std::numeric_limits<PixelType>::min();
    const Real max       = std::numeric_limits<PixelType>::max();
    const Real max_angle = math::pi<Real>;  // Bearing angle is max 180°

    return std::make_pair((max - min) / max_angle, -min);
}

/// This template calculates the angular resolution of the depth image
/// depending on the direction (\sa bearing_direction) and intrinsic parameters.
/// Every specialization of it computes the angular resolution differently.
template <bearing_direction Direction>
struct calculate_dphi {
    double operator()(const camera_models::pinhole_parameters &c) const
        noexcept;
};

template <>
double calculate_dphi<bearing_direction::horizontal>::
       operator()(const camera_models::pinhole_parameters &c) const noexcept {
    return c.x_resolution();
}
template <>
double calculate_dphi<bearing_direction::vertical>::
       operator()(const camera_models::pinhole_parameters &c) const noexcept {
    return c.y_resolution();
}

template <>
double calculate_dphi<bearing_direction::diagonal>::
       operator()(const camera_models::pinhole_parameters &c) const noexcept {
    return std::sqrt(c.x_resolution() * c.x_resolution() +
                     c.y_resolution() * c.y_resolution());
}
template <>
double calculate_dphi<bearing_direction::antidiagonal>::
       operator()(const camera_models::pinhole_parameters &c) const noexcept {
    return std::sqrt(c.x_resolution() * c.x_resolution() +
                     c.y_resolution() * c.y_resolution());
}

constexpr int get_dx(bearing_direction dir) {
    switch (dir) {
    case bearing_direction::antidiagonal:
    case bearing_direction::diagonal:
    case bearing_direction::horizontal: return -1;
    case bearing_direction::vertical: return 0;
    }
}
constexpr int get_dy(bearing_direction dir) {
    switch (dir) {
    case bearing_direction::antidiagonal: return 1;
    case bearing_direction::diagonal:
    case bearing_direction::vertical: return -1;
    case bearing_direction::horizontal: return 0;
    }
}

/// Provide a general accessor for the pixels that provide depth information
/// to calculate the bearing angle.
template <typename PixelType,
          bearing_direction Direction>  // requires CVInteger<PixelType>
class pixel {
  public:
    pixel(const cv::Mat &depth_image)
        : _depth_image{depth_image}
        , _dx{get_dx(Direction)}
        , _dy{get_dy(Direction)} {}
    PixelType operator()(int x, int y) const noexcept {
        return _depth_image.at<PixelType>(y + _dy, x + _dx);
    }

  private:
    const cv::Mat &_depth_image;
    const int      _dx = 0;
    const int      _dy = 0;
};

constexpr int get_x_start(bearing_direction dir) {
    switch (dir) {
    case bearing_direction::horizontal:
    case bearing_direction::antidiagonal:
    case bearing_direction::diagonal: return 1;
    case bearing_direction::vertical: return 0;
    }
}
constexpr int get_y_start(bearing_direction dir) {
    switch (dir) {
    case bearing_direction::horizontal:
    case bearing_direction::antidiagonal: return 0;
    case bearing_direction::diagonal:
    case bearing_direction::vertical: return 1;
    }
}
constexpr int get_dy_end(bearing_direction dir) {
    switch (dir) {
    case bearing_direction::antidiagonal: return 1;
    case bearing_direction::horizontal:
    case bearing_direction::diagonal:
    case bearing_direction::vertical: return 0;
    }
}

/// Define the start and end iterator for each bearing angle direction.
template <bearing_direction Direction>
struct range {
    range(const cv::Mat &depth_image) noexcept
        : x_start{get_x_start(Direction)}
        , x_end{depth_image.cols}
        , y_start{get_y_start(Direction)}
        , y_end{depth_image.rows - get_dy_end(Direction)} {}

    const int x_start;
    const int x_end;
    const int y_start;
    const int y_end;
};

template <typename Number>  // requires Number<Number>
int get_f_type() {
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

template <bearing_direction Direction, typename Real = float,
          typename PixelType = ushort>
// requires Float<Real> && CVIntegerPixelType<PixelType>
inline cv::Mat
depth_to_bearing(const cv::Mat &                          depth_image,
                 const camera_models::pinhole_parameters &intrinsic) noexcept {
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    using namespace detail;

    const double res{calculate_dphi<Direction>{}(intrinsic)};
    const Real   d_phi{gsl::narrow_cast<Real>(res)};
    const Real   cos_dphi{gsl::narrow_cast<Real>(std::cos(d_phi))};

    const pixel<PixelType, Direction> prior_accessor{depth_image};
    const range<Direction>            r{depth_image};

    Expects(d_phi > 0.);

    // Image of Reals, that will be converted after the full calculation.
    cv::Mat ba_image(depth_image.rows, depth_image.cols, get_f_type<Real>());
    Ensures(ba_image.channels() == 1);

    for (int y = r.y_start; y < r.y_end; ++y) {
        for (int x = r.x_start; x < r.x_end; ++x) {
            const PixelType d_i{depth_image.at<PixelType>(y, x)};
            const PixelType d_j{prior_accessor(x, y)};

            // A depth==0 means there is no measurement at this pixel.
            const Real angle =
                (d_i == 0 || d_j == 0)
                    ? 0.
                    : ba_formula(gsl::narrow_cast<Real>(d_i),
                                 gsl::narrow_cast<Real>(d_j), cos_dphi, d_phi);

            ba_image.at<Real>(y, x) = angle;
        }
    }
    cv::Mat return_image(depth_image.rows, depth_image.cols,
                         depth_image.type());
    auto [scale, offset] = scaling_factor<Real, PixelType>();
    ba_image.convertTo(return_image, get_f_type<PixelType>(), scale, offset);

    Ensures(return_image.channels() == 1);
    Ensures(return_image.type() == return_image.type());
    Ensures(!return_image.empty());
    Ensures(return_image.rows == return_image.rows);
    Ensures(return_image.cols == return_image.cols);

    return return_image;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_BEARING_H_ZXFA9HGG */
