#ifndef DEPTH_TO_CURVATURE_H_EK8HDTN0
#define DEPTH_TO_CURVATURE_H_EK8HDTN0

#include <algorithm>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/curvature.h>
#include <sens_loc/math/derivatives.h>
#include <sens_loc/math/scaling.h>

namespace sens_loc { namespace conversion {

/// Convert the range image \p depth_image to a gaussian curvature image.
///
/// The gaussian curvature is a measure of curvature of surfaces and intrinsic
/// to the inner geometry. See https://en.wikipedia.org/wiki/Gaussian_curvature
/// for more information.
///
/// \tparam Real precision of the calculation
/// \tparam PixelType underlying type of \p depth_image
/// \param depth_image range image that will be converted
/// \param intrinsic calibration of the sensor that took the image.
/// \returns estimated gaussian curvature for each pixel as \p Real
/// \pre \p depth_image is not empty
/// \pre \p PixelType matches the underlying type of \p depth_image
/// \note invalid depth values result in 0 for that pixel.
/// \sa depth_to_laserscan
template <typename Real = float, typename PixelType = float>
cv::Mat
depth_to_gaussian_curvature(const cv::Mat &               depth_image,
                            const camera_models::pinhole &intrinsic) noexcept;

/// Convert the range image \p depth_image to a mean curvature image.
///
/// Mean curvature is the second most used quantity for curvature in
/// differential geometry. See https://en.wikipedia.org/wiki/Mean_curvature
/// for more information.
///
/// \tparam Real precision of the calculation
/// \tparam PixelType underlying type of \p depth_image
/// \param depth_image range image that will be converted
/// \param intrinsic calibration of the sensor that took the image.
/// \returns estimated mean curvature for each pixel as \p Real
/// \pre \p depth_image is not empty
/// \pre \p PixelType matches the underlying type of \p depth_image
/// \note invalid depth values result in 0 for that pixel.
/// \sa depth_to_laserscan
template <typename Real = float, typename PixelType = float>
cv::Mat
depth_to_mean_curvature(const cv::Mat &               depth_image,
                        const camera_models::pinhole &intrinsic) noexcept;

/// Convert the curvature images to presentable images.
/// 
/// The issue with the curvature images is that the result can be any real
/// number. Together with the noisy input this result in gray images, because
/// the classical integer types can not represent them well.
/// This function can clamp the values to \f$[clamp_{min}, clamp_{max}]\f$ and
/// create images that still contain information.
/// If the clamping is off, the final images are probably not well suited for
/// classical computer vision tasks like feature matching.
///
/// \tparam Real underlying type of \p curvature_img
/// \tparam PixelType underlying type of the final converted image
/// \param curvature_img curvature image to convert
/// \param depth_image_as_mask invalid values are a 0 in the original
/// depth image. Because the curvature can not reflect this well the mask
/// is there to identify invalid depth values and therefore invalid curvatures
/// \param clamp_min,clamp_max target range of the values to scale to.
/// \returns converted image of some integertype (\p PixelType) that can be
/// used for feature matching.
/// \sa conversion::depth_to_mean_curvature
/// \sa conversion::depth_to_gaussian_curvature
/// \note both boundaries dont need to be provided!
/// \post range of each pixel is either
/// \f$[min(curvature_img), max(curvature_img)\f$ or
/// \f$[clamp_{min}, clamp_{max}]\f$ or any combination of the range limits.
template <typename Real = double, typename PixelType = ushort>
cv::Mat
curvature_to_image(const cv::Mat &     curvature_img,
                   const cv::Mat &     depth_image_as_mask,
                   std::optional<Real> clamp_min = std::nullopt,
                   std::optional<Real> clamp_max = std::nullopt) noexcept;

namespace detail {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DIFF_STAR(depth_image, curv_image)                                     \
    const Real d__1__1 = (depth_image).at<PixelType>(v - 1, u - 1);            \
    const Real d__1__0 = (depth_image).at<PixelType>(v - 1, u);                \
    const Real d__1_1  = (depth_image).at<PixelType>(v - 1, u + 1);            \
                                                                               \
    const Real d__0__1 = (depth_image).at<PixelType>(v, u - 1);                \
    const Real d__0__0 = (depth_image).at<PixelType>(v, u);                    \
    const Real d__0_1  = (depth_image).at<PixelType>(v, u + 1);                \
                                                                               \
    const Real d_1__1 = (depth_image).at<PixelType>(v + 1, u - 1);             \
    const Real d_1__0 = (depth_image).at<PixelType>(v + 1, u);                 \
    const Real d_1_1  = (depth_image).at<PixelType>(v + 1, u + 1);             \
                                                                               \
    if (d__1__1 == 0. || d__1__0 == 0. || d__1_1 == 0. || d__0__1 == 0. ||     \
        d__0__0 == 0. || d__0_1 == 0. || d_1__1 == 0. || d_1__0 == 0. ||       \
        d_1_1 == 0.) {                                                         \
        (curv_image).at<Real>(v, u) = 0.;                                      \
        continue;                                                              \
    }

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
template <typename Real, typename PixelType>
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
template <typename Real, typename PixelType>
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

namespace detail {

/// Convert a curvature image to an image. This will scale the reals that
/// are calculated to an range that is representable in normal images,
/// preferably to 16bit unsigned integers.
template <typename Real, typename PixelType = ushort>
inline cv::Mat
reals_to_image(const cv::Mat &     real_image,
               std::optional<Real> clamp_min = std::nullopt,
               std::optional<Real> clamp_max = std::nullopt) noexcept {
    Expects(real_image.type() == detail::get_cv_type<float>() ||
            real_image.type() == detail::get_cv_type<double>());
    Expects(real_image.cols > 0);
    Expects(real_image.rows > 0);

    // FIXME: Optimization opportunity: use min_max_element instead to save
    // one iteration over all values of \par real_image.
    const Real source_min = clamp_min
                                ? *clamp_min
                                : *std::min_element(real_image.begin<Real>(),
                                                    real_image.end<Real>());
    const Real source_max = clamp_max
                                ? *clamp_max
                                : *std::max_element(real_image.begin<Real>(),
                                                    real_image.end<Real>());

    const Real target_min = std::numeric_limits<PixelType>::min();
    const Real target_max = std::numeric_limits<PixelType>::max();

    cv::Mat target_image(real_image.rows, real_image.cols,
                         detail::get_cv_type<PixelType>());

    std::transform(real_image.begin<Real>(), real_image.end<Real>(),
                   target_image.begin<PixelType>(), [&](Real value) {
                       return gsl::narrow_cast<PixelType>(
                           math::scale({source_min, source_max},
                                       {target_min, target_max}, value));
                   });

    Ensures(target_image.cols == real_image.cols);
    Ensures(target_image.rows == real_image.rows);

    return target_image;
}

}  // namespace detail

template <typename Real, typename PixelType>
inline cv::Mat curvature_to_image(const cv::Mat &     curvature_img,
                                  const cv::Mat &     depth_image_as_mask,
                                  std::optional<Real> clamp_min,
                                  std::optional<Real> clamp_max) noexcept {
    Expects(curvature_img.type() == detail::get_cv_type<float>() ||
            curvature_img.type() == detail::get_cv_type<double>());
    Expects(curvature_img.cols == depth_image_as_mask.cols);
    Expects(curvature_img.rows == depth_image_as_mask.rows);

    cv::Mat mask_from_depth;
    depth_image_as_mask.convertTo(mask_from_depth, CV_8U);

    cv::Mat intermediate;
    detail::reals_to_image<Real>(curvature_img, clamp_min, clamp_max)
        .convertTo(intermediate, detail::get_cv_type<PixelType>());

    cv::Mat result(curvature_img.rows, curvature_img.cols,
                   detail::get_cv_type<PixelType>());
    intermediate.copyTo(result, mask_from_depth);

    Ensures(result.cols == curvature_img.cols);
    Ensures(result.rows == curvature_img.rows);
    Ensures(result.type() == detail::get_cv_type<PixelType>());
    Ensures(result.channels() == 1);

    return result;
}

}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_CURVATURE_H_EK8HDTN0 */
