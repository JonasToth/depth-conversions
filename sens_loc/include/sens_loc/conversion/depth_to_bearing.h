#ifndef DEPTH_TO_BEARING_H_ZXFA9HGG
#define DEPTH_TO_BEARING_H_ZXFA9HGG

#include <cmath>
#include <gsl/gsl>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/triangles.h>
#include <sens_loc/util/correctness_util.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc { namespace conversion {

/// Convert the image \param depth_image to an bearing angle image.
/// This function returns a new image with the same dimension as
/// \param depth_image but each pixel value is the corresponding bearing angle.
//
/// \note Depth images are orthografic and require conversion first!
///
/// \Expects \param depth_image to have 1 channel
/// \Expects \param depth_image == laser-scan like image!
/// \returns cv::Mat<Real> with each pixel beeing either 0 (no value) or the
/// bearing angle in radians (0, PI).
/// \sa convert_bearing
template <direction Direction, typename Real = float,
          typename PixelType = float>
cv::Mat depth_to_bearing(const cv::Mat &               depth_image,
                         const camera_models::pinhole &intrinsic) noexcept;

/// This function provides are parallelized version of the conversion
/// functionality.
///
/// The interface is different and expects different things then the normal
/// depth_to_bearing.
template <direction Direction, typename Real = float,
          typename PixelType = float>
std::pair<tf::Task, tf::Task>
par_depth_to_bearing(const cv::Mat &               depth_image,
                     const camera_models::pinhole &intrinsic, cv::Mat &ba_image,
                     tf::Taskflow &flow) noexcept;

/// Convert a bearing angle image to an image with integer types.
/// This function scales the bearing angles between
/// [PixelType::min, PixelType::max] for the angles in range (0, PI).
template <typename Real = float, typename PixelType = ushort>
cv::Mat convert_bearing(const cv::Mat &bearing_image) noexcept;

namespace detail {
inline int get_du(direction dir) {
    switch (dir) {
    case direction::antidiagonal:
    case direction::diagonal:
    case direction::horizontal: return -1;
    case direction::vertical: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");
}

inline int get_dv(direction dir) {
    switch (dir) {
    case direction::antidiagonal: return 1;
    case direction::diagonal:
    case direction::vertical: return -1;
    case direction::horizontal: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");
}

/// Provide a general accessor for the pixels that provide depth information
/// to calculate the bearing angle.
template <direction Direction>  // requires CVInteger<PixelType>
class pixel {
  public:
    pixel()
        : _du{get_du(Direction)}
        , _dv{get_dv(Direction)} {}
    /// \returns (u, v) for the prior point depending on the direction.
    std::pair<int, int> operator()(int u, int v) const noexcept {
        return std::make_pair(u + _du, v + _dv);
    }

  private:
    const int _du = 0;
    const int _dv = 0;
};

inline int get_x_start(direction dir) {
    switch (dir) {
    case direction::horizontal:
    case direction::antidiagonal:
    case direction::diagonal: return 1;
    case direction::vertical: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");
}

inline int get_y_start(direction dir) {
    switch (dir) {
    case direction::horizontal:
    case direction::antidiagonal: return 0;
    case direction::diagonal:
    case direction::vertical: return 1;
    }
    UNREACHABLE("Only 4 directions are possible");
}

inline int get_dy_end(direction dir) {
    switch (dir) {
    case direction::antidiagonal: return 1;
    case direction::horizontal:
    case direction::diagonal:
    case direction::vertical: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");
}

/// Define the start and end iterator for each bearing angle direction.
template <direction Direction>
struct pixel_range {
    pixel_range(const cv::Mat &depth_image) noexcept
        : x_start{get_x_start(Direction)}
        , x_end{depth_image.cols}
        , y_start{get_y_start(Direction)}
        , y_end{depth_image.rows - get_dy_end(Direction)} {}

    const int x_start;
    const int x_end;
    const int y_start;
    const int y_end;
};

template <typename Real, typename PixelType, typename RangeLimits,
          typename PriorAccess>
inline void
bearing_inner(const RangeLimits &r, const PriorAccess &prior_accessor,
              const int v, const cv::Mat &depth_image,
              const camera_models::pinhole &intrinsic, cv::Mat &ba_image) {
    for (int u = r.x_start; u < r.x_end; ++u) {
        const auto [u_p, v_p] = prior_accessor(u, v);

        const PixelType d_i{depth_image.at<PixelType>(v, u)};
        const PixelType d_j{depth_image.at<PixelType>(v_p, u_p)};

        Expects(d_i >= 0.);
        Expects(d_j >= 0.);

        const Real d_phi{gsl::narrow_cast<Real>(intrinsic.phi(u, v, u_p, v_p))};

        // A depth==0 means there is no measurement at this pixel.
        const Real angle =
            (d_i == 0. || d_j == 0.)
                ? 0.
                : math::bearing_angle(
                      gsl::narrow_cast<Real>(depth_image.at<PixelType>(v, u)),
                      gsl::narrow_cast<Real>(
                          depth_image.at<PixelType>(v_p, u_p)),
                      std::cos(d_phi));

        Ensures(angle >= 0.);
        Ensures(angle < math::pi<Real>);

        ba_image.at<Real>(v, u) = angle;
    }
}
}  // namespace detail

template <direction Direction, typename Real, typename PixelType>
// requires Float<Real> && CVIntegerPixelType<PixelType>
inline cv::Mat
depth_to_bearing(const cv::Mat &               depth_image,
                 const camera_models::pinhole &intrinsic) noexcept {
    using namespace detail;

    Expects(depth_image.type() == get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    const pixel<Direction>       prior_accessor;
    const pixel_range<Direction> r{depth_image};

    // Image of Reals, that will be converted after the full calculation.
    cv::Mat ba_image(depth_image.rows, depth_image.cols, get_cv_type<Real>());
    Ensures(ba_image.channels() == 1);

    for (int v = r.y_start; v < r.y_end; ++v) {
        detail::bearing_inner<Real, PixelType>(
            r, prior_accessor, v, depth_image, intrinsic, ba_image);
    }

    Ensures(ba_image.channels() == 1);
    Ensures(ba_image.type() == get_cv_type<Real>());
    Ensures(!ba_image.empty());
    Ensures(ba_image.rows == depth_image.rows);
    Ensures(ba_image.cols == depth_image.cols);

    return ba_image;
}

template <direction Direction, typename Real, typename PixelType>
inline std::pair<tf::Task, tf::Task>
par_depth_to_bearing(const cv::Mat &               depth_image,
                     const camera_models::pinhole &intrinsic, cv::Mat &ba_image,
                     tf::Taskflow &flow) noexcept {
    using namespace detail;
    Expects(depth_image.type() == get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);
    Expects(ba_image.cols == depth_image.cols);
    Expects(ba_image.rows == depth_image.rows);
    Expects(ba_image.channels() == depth_image.channels());
    Expects(ba_image.type() == get_cv_type<Real>());

    const pixel<Direction>       prior_accessor;
    const pixel_range<Direction> r{depth_image};

    auto sync_points = flow.parallel_for(
        r.y_start, r.y_end, 1,
        [prior_accessor, r, &depth_image, &intrinsic, &ba_image](int v) noexcept {
            detail::bearing_inner<Real, PixelType>(
                r, prior_accessor, v, depth_image, intrinsic, ba_image);
        });

    return sync_points;
}

template <typename Real, typename PixelType>
inline cv::Mat convert_bearing(const cv::Mat &bearing_image) noexcept {
    using namespace detail;

    Expects(bearing_image.channels() == 1);
    Expects(bearing_image.rows > 2);
    Expects(bearing_image.cols > 2);
    Expects(bearing_image.type() == get_cv_type<Real>());

    cv::Mat img(bearing_image.rows, bearing_image.cols,
                get_cv_type<PixelType>());
    auto [scale, offset] =
        scaling_factor<Real, PixelType>(/*max_angle = */ math::pi<Real>);
    bearing_image.convertTo(img, get_cv_type<PixelType>(), scale, offset);

    Ensures(img.cols == bearing_image.cols);
    Ensures(img.rows == bearing_image.rows);
    Ensures(img.type() == get_cv_type<PixelType>());
    Ensures(img.channels() == 1);

    return img;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_BEARING_H_ZXFA9HGG */
