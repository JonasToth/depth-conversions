#ifndef DEPTH_TO_CURVATURE_H_EK8HDTN0
#define DEPTH_TO_CURVATURE_H_EK8HDTN0

#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/curvature.h>
#include <sens_loc/math/derivatives.h>

namespace sens_loc { namespace conversion {

#define DIFF_STAR(depth_image, curv_image)                                     \
    const Real d__1__1 = depth_image.at<PixelType>(v - 1, u - 1);              \
    const Real d__1__0 = depth_image.at<PixelType>(v - 1, u);                  \
    const Real d__1_1  = depth_image.at<PixelType>(v - 1, u + 1);              \
                                                                               \
    const Real d__0__1 = depth_image.at<PixelType>(v, u - 1);                  \
    const Real d__0__0 = depth_image.at<PixelType>(v, u);                      \
    const Real d__0_1  = depth_image.at<PixelType>(v, u + 1);                  \
                                                                               \
    const Real d_1__1 = depth_image.at<PixelType>(v + 1, u - 1);               \
    const Real d_1__0 = depth_image.at<PixelType>(v + 1, u);                   \
    const Real d_1_1  = depth_image.at<PixelType>(v + 1, u + 1);               \
                                                                               \
    if (d__1__1 == 0. || d__1__0 == 0. || d__1_1 == 0. || d__0__1 == 0. ||     \
        d__0__0 == 0. || d__0_1 == 0. || d_1__1 == 0. || d_1__0 == 0. ||       \
        d_1_1 == 0.) {                                                         \
        curv_image.at<Real>(v, u) = 0.;                                        \
        continue;                                                              \
    }

namespace detail {
template <typename Real, typename PixelType>
void gaussian_inner(const int v, const cv::Mat &depth_image,
                    const camera_models::pinhole &intrinsic,
                    cv::Mat &                     target_img) {
    for (int u = 1; u < depth_image.cols - 1; ++u) {
        DIFF_STAR(depth_image, target_img)
        const Real d_phi       = intrinsic.phi(u - 1, v, u + 1, v);
        const Real d_theta     = intrinsic.phi(u, v - 1, u, v + 1);
        const Real d_phi_theta = intrinsic.phi(u - 1, v - 1, u + 1, v + 1);

        const auto [f_u, f_v, f_uu, f_vv, f_uv] = math::derivatives(
            d__1__1, d__1__0, d__1_1, d__0__1, d__0__0, d__0_1, d_1__1, d_1__0,
            d_1_1, d_phi, d_theta, d_phi_theta);

        const Real K = math::gaussian_curvature(f_u, f_v, f_uu, f_vv, f_uv);
        target_img.at<Real>(v, u) = K;
    }
}

template <typename Real, typename PixelType>
void mean_inner(const int v, const cv::Mat &depth_image,
                const camera_models::pinhole &intrinsic, cv::Mat &target_img) {
    for (int u = 1; u < depth_image.cols - 1; ++u) {
        DIFF_STAR(depth_image, target_img)

        const Real d_phi       = intrinsic.phi(u - 1, v, u + 1, v);
        const Real d_theta     = intrinsic.phi(u, v - 1, u, v + 1);
        const Real d_phi_theta = intrinsic.phi(u - 1, v - 1, u + 1, v + 1);

        const auto [f_u, f_v, f_uu, f_vv, f_uv] = math::derivatives(
            d__1__1, d__1__0, d__1_1, d__0__1, d__0__0, d__0_1, d_1__1, d_1__0,
            d_1_1, d_phi, d_theta, d_phi_theta);

        const Real K = math::mean_curvature(f_u, f_v, f_uu, f_vv, f_uv);
        target_img.at<Real>(v, u) = K;
    }
}
#undef DIFF_STAR

}  // namespace detail

/// Convert an euclidian depth image to a gaussian curvature image.
template <typename Real = float, typename PixelType = float>
inline cv::Mat
depth_to_gaussian_curvature(const cv::Mat &               depth_image,
                            const camera_models::pinhole &intrinsic) noexcept {

    Expects(depth_image.type() == detail::get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    cv::Mat gauss(depth_image.rows, depth_image.cols,
                  detail::get_cv_type<Real>());

    for (int v = 1; v < depth_image.rows - 1; ++v) {
        detail::gaussian_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                gauss);
    }
    Ensures(gauss.cols == depth_image.cols);
    Ensures(gauss.rows == depth_image.rows);
    Ensures(gauss.type() == detail::get_cv_type<Real>());
    Ensures(gauss.channels() == 1);

    return gauss;
}

/// Convert an euclidian depth image to a gaussian curvature image.
template <typename Real = float, typename PixelType = float>
inline cv::Mat
depth_to_mean_curvature(const cv::Mat &               depth_image,
                        const camera_models::pinhole &intrinsic) noexcept {

    Expects(depth_image.type() == detail::get_cv_type<PixelType>());
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    cv::Mat mean(depth_image.rows, depth_image.cols,
                 detail::get_cv_type<Real>());

    for (int v = 1; v < depth_image.rows - 1; ++v) {
        detail::mean_inner<Real, PixelType>(v, depth_image, intrinsic, mean);
    }
    Ensures(mean.cols == depth_image.cols);
    Ensures(mean.rows == depth_image.rows);
    Ensures(mean.type() == detail::get_cv_type<Real>());
    Ensures(mean.channels() == 1);

    return mean;
}


}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_CURVATURE_H_EK8HDTN0 */
