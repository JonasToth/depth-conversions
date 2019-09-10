#ifndef DEPTH_TO_TRIPLE_H_Y021ENVZ
#define DEPTH_TO_TRIPLE_H_Y021ENVZ

#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>

namespace sens_loc { namespace conversion {
namespace detail {
std::tuple<double, double, double>
add(std::tuple<double, double, double> v0,
    std::tuple<double, double, double> v1) noexcept {
    using std::get;
    return std::make_tuple(get<0>(v0) + get<0>(v1), get<1>(v0) + get<1>(v1),
                           get<2>(v0) + get<2>(v1));
}
std::tuple<double, double, double>
minus(std::tuple<double, double, double> v0,
      std::tuple<double, double, double> v1) noexcept {
    using std::get;
    return std::make_tuple(get<0>(v0) - get<0>(v1), get<1>(v0) - get<1>(v1),
                           get<2>(v0) - get<2>(v1));
}
std::tuple<double, double, double>
cross(std::tuple<double, double, double> v0,
      std::tuple<double, double, double> v1) noexcept {
    using std::get;
    return std::make_tuple(get<1>(v0) * get<2>(v1) - get<2>(v0) * get<1>(v1),
                           get<2>(v0) * get<0>(v1) - get<0>(v0) * get<2>(v1),
                           get<0>(v0) * get<1>(v1) - get<1>(v0) * get<0>(v1));
}
double dot(std::tuple<double, double, double> v0,
           std::tuple<double, double, double> v1) noexcept {
    using std::get;
    return get<0>(v0) * get<0>(v1) + get<1>(v0) * get<1>(v1) +
           get<2>(v0) * get<2>(v1);
}

double len(std::tuple<double, double, double> v) noexcept {
    const auto [x, y, z] = v;
    return std::sqrt(x * x + y * y + z * z);
}

std::tuple<double, double, double>
norm(std::tuple<double, double, double> v) noexcept {
    const auto l = len(v);
    using std::get;
    return std::make_tuple(get<0>(v) / l, get<1>(v) / l, get<2>(v) / l);
}

std::tuple<double, double, double>
get_cartesian(const camera_models::pinhole &intrinsic, int u, int v,
              double d) noexcept {
    const auto [xs, ys, zs] = intrinsic.project_to_sphere(u, v);
    return std::make_tuple(d * xs, d * ys, d * zs);
}

template <typename Real = float, typename PixelType = float>
inline void triple_inner(int v, const cv::Mat &depth_image,
                         const camera_models::pinhole &intrinsic,
                         cv::Mat &                     out) {
    for (int u = 1; u < depth_image.cols - 1; ++u) {
        const Real d__1__0 = depth_image.at<PixelType>(v - 1, u);
        const Real d_1__0  = depth_image.at<PixelType>(v + 1, u);

        const Real d__0__1 = depth_image.at<PixelType>(v, u - 1);
        const Real d__0_1  = depth_image.at<PixelType>(v, u + 1);

        const Real d__1_1 = depth_image.at<PixelType>(v - 1, u + 1);
        const Real d_1__1  = depth_image.at<PixelType>(v + 1, u - 1);

        const Real d__1__1 = depth_image.at<PixelType>(v - 1, u - 1);
        const Real d_1_1  = depth_image.at<PixelType>(v + 1, u + 1);

        if (d__1__0 == 0. || d__0__1 == 0. || d_1__0 == 0. || d__0_1 == 0.) {
            out.at<Real>(v, u) = 0.;
            continue;
        }

        const auto surface_pt0  = get_cartesian(intrinsic, v - 1, u, d__1__0);
        const auto surface_pt1  = get_cartesian(intrinsic, v + 1, u, d_1__0);
        const auto surface_dir0 = minus(surface_pt1, surface_pt0);

        const auto surface_pt2  = get_cartesian(intrinsic, v, u - 1, d__0__1);
        const auto surface_pt3  = get_cartesian(intrinsic, v, u + 1, d__0_1);
        const auto surface_dir1 = minus(surface_pt3, surface_pt2);
 
        const auto surface_pt4  = get_cartesian(intrinsic, v - 1, u + 1, d__1_1);
        const auto surface_pt5  = get_cartesian(intrinsic, v + 1, u - 1, d_1__1);
        const auto surface_dir2 = minus(surface_pt5, surface_pt4);

        const auto surface_pt6  = get_cartesian(intrinsic, v - 1, u - 1, d__1__1);
        const auto surface_pt7  = get_cartesian(intrinsic, v + 1, u + 1, d_1_1);
        const auto surface_dir3 = minus(surface_pt7, surface_pt6);

        // auto n = cross(surface_dir0, surface_dir1);
        const auto cross0 = cross(norm(surface_dir0), norm(surface_dir1));
        const auto cross1 = cross(norm(surface_dir2), norm(surface_dir3));

        // const auto triple = std::abs(dot(cross0, cross1));
        const auto triple = 255. * std::abs(dot(cross0, cross1));

        // auto triple = 255. * dot(norm(surface_dir0), norm(surface_dir1));
        // auto triple = 255. * dot(surface_dir0, surface_dir1);

        // auto triple  = std::abs(dot(cross(sphere_dir0, sphere_dir1), n));
        // auto triple  = 255. * dot(surface_dir0, surface_dir1);
        out.at<Real>(v, u) = triple;
    }
}

}  // namespace detail

/// Convert an euclidian depth image to a triple-product-image.
template <typename Real = float, typename PixelType = float>
inline cv::Mat
depth_to_triple(const cv::Mat &               depth_image,
                const camera_models::pinhole &intrinsic) noexcept {
    Expects(depth_image.type() == detail::get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    cv::Mat triple(depth_image.rows, depth_image.cols,
                   detail::get_cv_type<Real>());
    for (int v = 1; v < depth_image.rows - 1; ++v) {
        detail::triple_inner<Real, PixelType>(v, depth_image, intrinsic,
                                              triple);
    }

    Ensures(triple.cols == depth_image.cols);
    Ensures(triple.rows == depth_image.rows);
    Ensures(triple.type() == detail::get_cv_type<Real>());
    Ensures(triple.channels() == 1);

    return triple;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_TRIPLE_H_Y021ENVZ */
