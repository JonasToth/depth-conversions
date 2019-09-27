#ifndef DEPTH_TO_TRIPLE_H_Y021ENVZ
#define DEPTH_TO_TRIPLE_H_Y021ENVZ

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/eigen_types.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc { namespace conversion {
namespace detail {

using ::sens_loc::math::vec;

template <typename Real = float>
vec<Real, 3> get_cartesian(const camera_models::pinhole &intrinsic, int u,
                           int v, Real d) noexcept {
    const auto [xs, ys, zs] = intrinsic.project_to_sphere(u, v);
    return d * vec<Real, 3>(xs, ys, zs);
}

template <typename Real = float, typename PixelType = float>
inline void flexion_inner(int v, const cv::Mat &depth_image,
                          const camera_models::pinhole &intrinsic,
                          cv::Mat &                     out) {
    for (int u = 1; u < depth_image.cols - 1; ++u) {
        const Real d__1__0 = depth_image.at<PixelType>(v - 1, u);
        const Real d_1__0  = depth_image.at<PixelType>(v + 1, u);

        const Real d__0__1 = depth_image.at<PixelType>(v, u - 1);
        const Real d__0_1  = depth_image.at<PixelType>(v, u + 1);

        const Real d__1_1 = depth_image.at<PixelType>(v - 1, u + 1);
        const Real d_1__1 = depth_image.at<PixelType>(v + 1, u - 1);

        const Real d__1__1 = depth_image.at<PixelType>(v - 1, u - 1);
        const Real d_1_1   = depth_image.at<PixelType>(v + 1, u + 1);

        // If any of the depths is zero, the resulting vector will be the
        // null vector. This with then propagate through as zero and does not
        // induce any undefined behaviour or other problems.
        // Not short-circuiting results in easier vectorization / GPU
        // acceleration.

        const auto surface_pt0  = get_cartesian(intrinsic, v - 1, u, d__1__0);
        const auto surface_pt1  = get_cartesian(intrinsic, v + 1, u, d_1__0);
        const auto surface_dir0 = surface_pt1 - surface_pt0;

        const auto surface_pt2  = get_cartesian(intrinsic, v, u - 1, d__0__1);
        const auto surface_pt3  = get_cartesian(intrinsic, v, u + 1, d__0_1);
        const auto surface_dir1 = surface_pt3 - surface_pt2;

        const auto surface_pt4 = get_cartesian(intrinsic, v - 1, u + 1, d__1_1);
        const auto surface_pt5 = get_cartesian(intrinsic, v + 1, u - 1, d_1__1);
        const auto surface_dir2 = surface_pt5 - surface_pt4;

        const auto surface_pt6 =
            get_cartesian(intrinsic, v - 1, u - 1, d__1__1);
        const auto surface_pt7  = get_cartesian(intrinsic, v + 1, u + 1, d_1_1);
        const auto surface_dir3 = surface_pt7 - surface_pt6;

        const auto cross0 =
            surface_dir0.normalized().cross(surface_dir1.normalized());
        const auto cross1 =
            surface_dir2.normalized().cross(surface_dir3.normalized());

        const auto triple = std::abs(cross0.dot(cross1));

        Ensures(triple >= 0.);
        Ensures(triple <= 1.);

        out.at<Real>(v, u) = triple;
    }
}

}  // namespace detail

/// Convert an euclidian depth image to a flexion-image.
template <typename Real = float, typename PixelType = float>
inline cv::Mat
depth_to_flexion(const cv::Mat &               depth_image,
                 const camera_models::pinhole &intrinsic) noexcept {
    Expects(depth_image.type() == detail::get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    cv::Mat triple(depth_image.rows, depth_image.cols,
                   detail::get_cv_type<Real>());
    for (int v = 1; v < depth_image.rows - 1; ++v) {
        detail::flexion_inner<Real, PixelType>(v, depth_image, intrinsic,
                                               triple);
    }

    Ensures(triple.cols == depth_image.cols);
    Ensures(triple.rows == depth_image.rows);
    Ensures(triple.type() == detail::get_cv_type<Real>());
    Ensures(triple.channels() == 1);

    return triple;
}

/// Convert an euclidian depth image to a triple-product-image.
template <typename Real = float, typename PixelType = float>
inline std::pair<tf::Task, tf::Task>
par_depth_to_flexion(const cv::Mat &               depth_image,
                     const camera_models::pinhole &intrinsic,
                     cv::Mat &triple_image, tf::Taskflow &flow) noexcept {
    Expects(depth_image.type() == detail::get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);
    Expects(triple_image.cols == depth_image.cols);
    Expects(triple_image.rows == depth_image.rows);
    Expects(triple_image.channels() == depth_image.channels());
    Expects(triple_image.type() == detail::get_cv_type<Real>());

    auto sync_points = flow.parallel_for(
        1, depth_image.rows - 1, 1, [&](int v) noexcept {
            detail::flexion_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                   triple_image);
        });

    return sync_points;
}

/// Scale the image properly for image io.
template <typename Real, typename PixelType>
inline cv::Mat convert_flexion(const cv::Mat &flexion_image) noexcept {
    Expects(flexion_image.channels() == 1);
    Expects(flexion_image.rows > 2);
    Expects(flexion_image.cols > 2);
    Expects(flexion_image.type() == detail::get_cv_type<Real>());

    cv::Mat    img(flexion_image.rows, flexion_image.cols,
                detail::get_cv_type<PixelType>());
    const auto scale = std::numeric_limits<PixelType>::max() -
                       std::numeric_limits<PixelType>::min();
    const auto offset = std::numeric_limits<PixelType>::min();

    flexion_image.convertTo(img, detail::get_cv_type<PixelType>(), scale,
                            offset);

    Ensures(img.cols == flexion_image.cols);
    Ensures(img.rows == flexion_image.rows);
    Ensures(img.type() == detail::get_cv_type<PixelType>());
    Ensures(img.channels() == 1);

    return img;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_TRIPLE_H_Y021ENVZ */
