#ifndef DEPTH_TO_BEARING_H_ZXFA9HGG
#define DEPTH_TO_BEARING_H_ZXFA9HGG

#include <cmath>
#include <gsl/gsl>
#include <limits>
#include <sens_loc/camera_models/concepts.h>
#include <sens_loc/camera_models/utility.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/image.h>
#include <sens_loc/math/triangles.h>
#include <sens_loc/util/correctness_util.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc { namespace conversion {

/// Convert the image \p depth_image to an bearing angle image.
/// This function returns a new image with the same dimension as \p depth_image.
///
/// \tparam Direction the direction for neighbourhood-relationship between
/// pixels that form the bearing angle
/// \tparam Real precision to calculate in, needs to be floating-point
/// \tparam Intrinsic camera model that projects pixel to the unit sphere
/// \param depth_image range image that was taken by a sensor with the
/// calibration from \p intrinsic
/// \param intrinsic camera model to calculate the angle between light
/// rays that correspond to pixels
/// \returns cv::Mat<Real> with each pixel the bearing angle (radians) in
/// \p Direction. Invalid depth values result in 0 as result.
//
/// \note Depth images are orthografic and require conversion first!
/// \sa conversion::depth_to_laserscan
/// \sa camera_models::is_intrinsic_v
///
/// \pre \p depth_image to have 1 channel
/// \pre \p depth_image == laser-scan like image!
/// \pre \p depth_image is not empty
/// \pre \p the underlying type of \p depth_image matches \p PixelType
/// \pre \p intrinsic matches sensor that took the image, prior preprocessing
/// \post each bearing angle is in the range \f$[0, \pi)\f$
/// is of course possible with matching changes in the \p depth_image.
template <direction Direction,
          template <typename>
          typename Intrinsic,
          typename Real = float>
math::image<Real> depth_to_bearing(const math::image<Real>& depth_image,
                                   const Intrinsic<Real>&   intrinsic) noexcept;

/// This function provides are parallelized version of the conversion
/// functionality.
///
/// This function creates a taskflow for the row-wise parallel calculation
/// of the bearing angle image.
/// Only differences are documented here.
/// \param[in] depth_image,intrinsic the same
/// \param[out] ba_image result image that will be created with parallel
/// processing
/// \param[inout] flow parallel flow type that is used to parallelize the outer
/// for loop over all rows.
/// \returns synchronization points before and after the calculation of the
/// bearing angle image.
/// \sa depth_to_bearing
template <direction Direction,
          template <typename>
          typename Intrinsic,
          typename Real = float>
std::pair<tf::Task, tf::Task>
par_depth_to_bearing(const math::image<Real>& depth_image,
                     const Intrinsic<Real>&   intrinsic,
                     math::image<Real>&       ba_image,
                     tf::Taskflow&            flow) noexcept;

/// Convert a bearing angle image to an image with integer types.
/// This function scales the bearing angles between
/// [PixelType::min, PixelType::max] for the angles in range (0, PI).
///
/// \tparam Real underyling type of the \p bearing_image, floating-point
/// \tparam PixelType target type for the converted image that is returned,
/// arithmetic type (integer or floating-point)
/// \param bearing_image calculated
/// bearing angle image
/// \returns the bearing angle image is converted to a "normal" image that can
/// be displayed and stored with normal image visualization tools.
/// \pre the underlying type of \p bearing_image is \p Real
/// \pre range of pixel is in \p bearinge_image \f$[0, \pi)\f$
/// \post the underlying type of the result is \p PixelType
/// \post range of pixel in result is \f$[PixelType_{min}, PixelType_{max}]\f$
/// \sa conversion::depth_to_bearing
template <typename Real = float, typename PixelType = ushort>
math::image<PixelType>
convert_bearing(const math::image<Real>& bearing_image) noexcept;

namespace detail {
inline int get_du(direction dir) {
    switch (dir) {
    case direction::antidiagonal:
    case direction::diagonal:
    case direction::horizontal: return -1;
    case direction::vertical: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");  // LCOV_EXCL_LINE
}

inline int get_dv(direction dir) {
    switch (dir) {
    case direction::diagonal:
    case direction::vertical: return -1;
    case direction::horizontal: return 0;
    case direction::antidiagonal: return 1;
    }
    UNREACHABLE("Only 4 directions are possible");  // LCOV_EXCL_LINE
}

/// Provide a general accessor for the pixels that provide depth information
/// to calculate the bearing angle.
template <typename Real, direction Direction>
class pixel {
  public:
    pixel()
        : _du{get_du(Direction)}
        , _dv{get_dv(Direction)} {}
    /// \returns (u, v) for the prior point depending on the direction.
    math::pixel_coord<int> operator()(const math::pixel_coord<int>& p) const
        noexcept {
        return {p.u() + _du, p.v() + _dv};
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
    UNREACHABLE("Only 4 directions are possible");  // LCOV_EXCL_LINE
}

inline int get_y_start(direction dir) {
    switch (dir) {
    case direction::horizontal:
    case direction::antidiagonal: return 0;
    case direction::diagonal:
    case direction::vertical: return 1;
    }
    UNREACHABLE("Only 4 directions are possible");  // LCOV_EXCL_LINE
}

inline int get_dy_end(direction dir) {
    switch (dir) {
    case direction::antidiagonal: return 1;
    case direction::horizontal:
    case direction::diagonal:
    case direction::vertical: return 0;
    }
    UNREACHABLE("Only 4 directions are possible");  // LCOV_EXCL_LINE
}

/// Define the start and end iterator for each bearing angle direction.
template <direction Direction>
struct pixel_range {
    pixel_range(const cv::Mat& depth_image) noexcept
        : x_start{get_x_start(Direction)}
        , x_end{depth_image.cols}
        , y_start{get_y_start(Direction)}
        , y_end{depth_image.rows - get_dy_end(Direction)} {}

    const int x_start;
    const int x_end;
    const int y_start;
    const int y_end;
};

template <typename Real,
          typename RangeLimits,
          typename PriorAccess,
          template <typename>
          typename Intrinsic>
inline void bearing_inner(const RangeLimits&       r,
                          const PriorAccess&       prior_accessor,
                          const int                v,
                          const math::image<Real>& depth_image,
                          const Intrinsic<Real>&   intrinsic,
                          math::image<Real>&       ba_image) {
    for (int u = r.x_start; u < r.x_end; ++u) {
        const math::pixel_coord<int> central(u, v);
        const math::pixel_coord<int> prior = prior_accessor(central);

        const auto d_i = depth_image.at(central);
        const auto d_j = depth_image.at(prior);

        Expects(d_i >= Real(0.));
        Expects(d_j >= Real(0.));

        const Real d_phi = camera_models::phi(intrinsic, central, prior);

        // A depth==0 means there is no measurement at this pixel.
        const Real angle =
            (d_i == Real(0.) || d_j == Real(0.))
                ? Real(0.)
                : math::bearing_angle<Real>(d_i, d_j, std::cos(d_phi));

        Ensures(angle >= Real(0.));
        Ensures(angle < math::pi<Real>);

        ba_image.at(central) = angle;
    }
}
}  // namespace detail

template <direction Direction,
          template <typename>
          typename Intrinsic,
          typename Real>
inline math::image<Real>
depth_to_bearing(const math::image<Real>& depth_image,
                 const Intrinsic<Real>&   intrinsic) noexcept {
    static_assert(camera_models::is_intrinsic_v<Intrinsic, Real>);
    static_assert(std::is_floating_point_v<Real>);

    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());

    using namespace detail;
    const pixel<Real, Direction> prior_accessor;
    const pixel_range<Direction> r{depth_image.data()};

    // Image of Reals, that will be converted after the full calculation.
    cv::Mat ba(depth_image.h(), depth_image.w(),
               math::detail::get_opencv_type<Real>());
    ba = Real(0.);
    math::image<Real> ba_image(std::move(ba));

    for (int v = r.y_start; v < r.y_end; ++v)
        detail::bearing_inner(r, prior_accessor, v, depth_image, intrinsic,
                              ba_image);

    Ensures(ba_image.h() == depth_image.h());
    Ensures(ba_image.w() == depth_image.w());

    return ba_image;
}

template <direction Direction,
          template <typename>
          typename Intrinsic,
          typename Real>
inline std::pair<tf::Task, tf::Task>
par_depth_to_bearing(const math::image<Real>& depth_image,
                     const Intrinsic<Real>&   intrinsic,
                     math::image<Real>&       ba_image,
                     tf::Taskflow&            flow) noexcept {
    static_assert(camera_models::is_intrinsic_v<Intrinsic, Real>);
    static_assert(std::is_floating_point_v<Real>);

    using namespace detail;
    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());
    Expects(ba_image.w() == depth_image.w());
    Expects(ba_image.h() == depth_image.h());

    const pixel<Real, Direction> prior_accessor;
    const pixel_range<Direction> r{depth_image.data()};

    auto sync_points = flow.parallel_for(
        r.y_start, r.y_end, 1,
        [prior_accessor, r, &depth_image, &intrinsic, &ba_image](int v) {
            detail::bearing_inner(r, prior_accessor, v, depth_image, intrinsic,
                                  ba_image);
        });

    return sync_points;
}

template <typename Real, typename PixelType>
inline math::image<PixelType>
convert_bearing(const math::image<Real>& bearing_image) noexcept {
    static_assert(std::is_floating_point_v<Real>);
    static_assert(std::is_arithmetic_v<PixelType>);

    using detail::scaling_factor;
    cv::Mat img(bearing_image.h(), bearing_image.w(),
                math::detail::get_opencv_type<PixelType>());
    auto [scale, offset] =
        scaling_factor<Real, PixelType>(/*max_angle = */ math::pi<Real>);
    bearing_image.data().convertTo(
        img, math::detail::get_opencv_type<PixelType>(), scale, offset);

    Ensures(img.cols == bearing_image.w());
    Ensures(img.rows == bearing_image.h());
    Ensures(img.type() == math::detail::get_opencv_type<PixelType>());
    Ensures(img.channels() == 1);

    return math::image<PixelType>(std::move(img));
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_BEARING_H_ZXFA9HGG */
