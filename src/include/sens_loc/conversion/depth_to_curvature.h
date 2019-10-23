#ifndef DEPTH_TO_CURVATURE_H_EK8HDTN0
#define DEPTH_TO_CURVATURE_H_EK8HDTN0

#include <algorithm>
#include <limits>
#include <optional>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/utility.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/curvature.h>
#include <sens_loc/math/derivatives.h>
#include <sens_loc/math/image.h>
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
math::image<Real> depth_to_gaussian_curvature(
    const math::image<PixelType> &      depth_image,
    const camera_models::pinhole<Real> &intrinsic) noexcept;

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
math::image<Real>
depth_to_mean_curvature(const math::image<PixelType> &      depth_image,
                        const camera_models::pinhole<Real> &intrinsic) noexcept;

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
template <typename Real = double, typename PixelType = ushort,
          typename MaskType = PixelType>
math::image<PixelType>
curvature_to_image(const math::image<Real> &    curvature_img,
                   const math::image<MaskType> &depth_image_as_mask,
                   std::optional<Real>          clamp_min = std::nullopt,
                   std::optional<Real> clamp_max = std::nullopt) noexcept;

namespace detail {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define DIFF_STAR(depth_image, curv_image)                                     \
    const Real d__1__1 = (depth_image).at({u - 1, v - 1});                     \
    const Real d__1__0 = (depth_image).at({u, v - 1});                         \
    const Real d__1_1  = (depth_image).at({u + 1, v - 1});                     \
                                                                               \
    const Real d__0__1 = (depth_image).at({u - 1, v});                         \
    const Real d__0__0 = (depth_image).at({u, v});                             \
    const Real d__0_1  = (depth_image).at({u + 1, v});                         \
                                                                               \
    const Real d_1__1 = (depth_image).at({u - 1, v + 1});                      \
    const Real d_1__0 = (depth_image).at({u, v + 1});                          \
    const Real d_1_1  = (depth_image).at({u + 1, v + 1});                      \
                                                                               \
    if (d__1__1 == 0. || d__1__0 == 0. || d__1_1 == 0. || d__0__1 == 0. ||     \
        d__0__0 == 0. || d__0_1 == 0. || d_1__1 == 0. || d_1__0 == 0. ||       \
        d_1_1 == 0.) {                                                         \
        (curv_image).at({u, v}) = 0.;                                          \
        continue;                                                              \
    }

template <typename Real, typename PixelType>
void gaussian_inner(const int v, const math::image<PixelType> &depth_image,
                    const camera_models::pinhole<Real> &intrinsic,
                    math::image<Real> &                 target_img) {
    for (int u = 1; u < depth_image.data().cols - 1; ++u) {
        DIFF_STAR(depth_image, target_img)

        using camera_models::phi;
        const Real d_phi       = phi(intrinsic, {u - 1, v}, {u + 1, v});
        const Real d_theta     = phi(intrinsic, {u, v - 1}, {u, v + 1});
        const Real d_phi_theta = phi(intrinsic, {u - 1, v - 1}, {u + 1, v + 1});

        const auto [f_u, f_v, f_uu, f_vv, f_uv] = math::derivatives(
            d__1__1, d__1__0, d__1_1, d__0__1, d__0__0, d__0_1, d_1__1, d_1__0,
            d_1_1, d_phi, d_theta, d_phi_theta);

        const Real K = math::gaussian_curvature(f_u, f_v, f_uu, f_vv, f_uv);
        target_img.at({u, v}) = K;
    }
}

template <typename Real, typename PixelType>
void mean_inner(const int v, const math::image<PixelType> &depth_image,
                const camera_models::pinhole<Real> &intrinsic,
                math::image<Real> &                 target_img) {
    for (int u = 1; u < depth_image.data().cols - 1; ++u) {
        DIFF_STAR(depth_image, target_img)

        using camera_models::phi;
        const Real d_phi       = phi(intrinsic, {u - 1, v}, {u + 1, v});
        const Real d_theta     = phi(intrinsic, {u, v - 1}, {u, v + 1});
        const Real d_phi_theta = phi(intrinsic, {u - 1, v - 1}, {u + 1, v + 1});

        const auto [f_u, f_v, f_uu, f_vv, f_uv] = math::derivatives(
            d__1__1, d__1__0, d__1_1, d__0__1, d__0__0, d__0_1, d_1__1, d_1__0,
            d_1_1, d_phi, d_theta, d_phi_theta);

        const Real K = math::mean_curvature(f_u, f_v, f_uu, f_vv, f_uv);
        target_img.at({u, v}) = K;
    }
}
#undef DIFF_STAR

}  // namespace detail

/// Convert an euclidian depth image to a gaussian curvature image.
template <typename Real, typename PixelType>
inline math::image<Real> depth_to_gaussian_curvature(
    const math::image<PixelType> &      depth_image,
    const camera_models::pinhole<Real> &intrinsic) noexcept {
    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());

    cv::Mat gauss(depth_image.data().rows, depth_image.data().cols,
                  math::detail::get_opencv_type<Real>());
    gauss = Real(0.);
    math::image<Real> gauss_image(std::move(gauss));

    for (int v = 1; v < depth_image.data().rows - 1; ++v) {
        detail::gaussian_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                gauss_image);
    }
    return gauss_image;
}

/// Convert an euclidian depth image to a gaussian curvature image.
template <typename Real, typename PixelType>
inline math::image<Real> depth_to_mean_curvature(
    const math::image<PixelType> &      depth_image,
    const camera_models::pinhole<Real> &intrinsic) noexcept {
    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());

    cv::Mat mean(depth_image.data().rows, depth_image.data().cols,
                 math::detail::get_opencv_type<Real>());
    mean = Real(0.);
    math::image<Real> mean_image(std::move(mean));

    for (int v = 1; v < depth_image.data().rows - 1; ++v) {
        detail::mean_inner<Real, PixelType>(v, depth_image, intrinsic,
                                            mean_image);
    }
    return mean_image;
}

namespace detail {

/// Convert a curvature image to an image. This will scale the reals that
/// are calculated to an range that is representable in normal images,
/// preferably to 16bit unsigned integers.
template <typename Real, typename PixelType = ushort>
inline math::image<PixelType>
reals_to_image(const math::image<Real> &real_image,
               std::optional<Real>      clamp_min = std::nullopt,
               std::optional<Real>      clamp_max = std::nullopt) noexcept {
    const auto [min_it, max_it] = [&]()
        -> std::pair<cv::MatConstIterator_<Real>, cv::MatConstIterator_<Real>> {
        if (!clamp_min || !clamp_max)
            return std::minmax_element(real_image.data().template begin<Real>(),
                                       real_image.data().template end<Real>());
        return {real_image.data().template end<Real>(),
                real_image.data().template end<Real>()};
    }();

    const Real source_min = clamp_min ? *clamp_min : *min_it;
    const Real source_max = clamp_max ? *clamp_max : *max_it;

    const Real target_min = std::numeric_limits<PixelType>::min();
    const Real target_max = std::numeric_limits<PixelType>::max();

    cv::Mat target_image(real_image.data().rows, real_image.data().cols,
                         math::detail::get_opencv_type<PixelType>());

    std::transform(real_image.data().template begin<Real>(),
                   real_image.data().template end<Real>(),
                   target_image.begin<PixelType>(), [&](Real value) {
                       return gsl::narrow_cast<PixelType>(
                           math::scale({source_min, source_max},
                                       {target_min, target_max}, value));
                   });

    Ensures(target_image.cols == real_image.data().cols);
    Ensures(target_image.rows == real_image.data().rows);

    return math::image<PixelType>(std::move(target_image));
}

}  // namespace detail

template <typename Real, typename PixelType, typename MaskType>
inline math::image<PixelType>
curvature_to_image(const math::image<Real> &    curvature_img,
                   const math::image<MaskType> &depth_image_as_mask,
                   std::optional<Real>          clamp_min,
                   std::optional<Real>          clamp_max) noexcept {
    Expects(curvature_img.data().cols == depth_image_as_mask.data().cols);
    Expects(curvature_img.data().rows == depth_image_as_mask.data().rows);

    cv::Mat mask_from_depth;
    depth_image_as_mask.data().convertTo(mask_from_depth, CV_8U);

    cv::Mat intermediate;
    detail::reals_to_image<Real>(curvature_img, clamp_min, clamp_max)
        .data()
        .convertTo(intermediate, math::detail::get_opencv_type<PixelType>());

    cv::Mat result(curvature_img.data().rows, curvature_img.data().cols,
                   math::detail::get_opencv_type<PixelType>());
    intermediate.copyTo(result, mask_from_depth);

    Ensures(result.cols == curvature_img.data().cols);
    Ensures(result.rows == curvature_img.data().rows);
    Ensures(result.type() == math::detail::get_opencv_type<PixelType>());
    Ensures(result.channels() == 1);

    return math::image<PixelType>(std::move(result));
}

}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_CURVATURE_H_EK8HDTN0 */
