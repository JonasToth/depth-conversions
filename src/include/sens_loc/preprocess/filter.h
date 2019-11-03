#ifndef FILTER_H_4UEYCWE1
#define FILTER_H_4UEYCWE1

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sens_loc/math/image.h>
#include <type_traits>
#include <variant>

namespace sens_loc {

/// This namespace contains functionality to preprocess depth/range-data.
/// It wraps functionality found in OpenCV (-contrib) and makes it accessable
/// for the strongly typed interfaces.
namespace preprocess {

/// This function wraps OpenCVs implementation of the bilateral filter for
/// single channel images.
///
/// See
/// https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed
/// for more information.
///
/// Sigma values: For simplicity, you can set the 2 sigma values to be the same.
/// If they are small (< 10), the filter will not have much effect, whereas if
/// they are large (> 150), they will have a very strong effect, making the
/// image look "cartoonish".
///
/// Filter size: Large filters (d > 5) are very slow, so it is recommended to
/// use d=5 for real-time applications, and perhaps d=9 for offline
/// applications that need heavy noise filtering.
///
/// This filter does not work inplace.
///
/// \note This function will convert \p input to \c image<float> before
/// running the filter.
/// \tparam PixelType underlying image type - note that internal conversion to
/// \c float will happen if \p PixelType is not \c float.
/// \param input input image that is going to be filtered
/// \param sigma_color Filter sigma in the color space. A larger value of the
/// parameter means that farther colors within the pixel neighborhood
/// (see \p sigmaSpace) will be mixed together, resulting in larger areas of
/// semi-equal color.
/// \param proximity Filter sigma in the coordinate space.
/// A larger value of the parameter means that farther pixels will influence
/// each other as long as their colors are close enough (see \p sigmaColor).
/// When an integer is provided the proximity is defined as the diameter in
/// pixels, if a double is provided the proximity is calculated from it.
template <typename PixelType>
math::image<PixelType>
bilateral_filter(const math::image<PixelType>& input,
                 double                        sigma_color,
                 std::variant<int, double>     proximity) noexcept {
    static_assert(std::is_arithmetic_v<PixelType>);

    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<float>(), 0.0F);

    const auto [distance,
                sigma_space] = [&proximity]() -> std::pair<int, double> {
        if (std::holds_alternative<int>(proximity))
            return std::make_pair(std::get<int>(proximity), 0.);
        return std::make_pair(0, std::get<double>(proximity));
    }();

    // The input is already in float format and does not require a prior
    // conversion.
    if constexpr (std::is_same_v<PixelType, float>) {
        cv::bilateralFilter(input.data(), filter_result, distance, sigma_color,
                            sigma_space);
        return math::image<float>(std::move(filter_result));
    }

    auto conv = math::convert<float>(input);
    cv::bilateralFilter(conv.data(), filter_result, distance, sigma_color,
                        sigma_space);
    auto res_pic = math::image<float>(std::move(filter_result));
    return math::convert<PixelType>(res_pic);
};


/// This function wraps OpenCVs implementation of the guided filter for
/// single channel images. (It does not produce good results...)
///
/// See
/// https://docs.opencv.org/master/da/d17/group__ximgproc__filters.html#ga86813d59f8452a9600e979c6287805f5
/// for more information.
template <typename PixelType>
math::image<PixelType> guided_filter(const math::image<PixelType>& input,
                                     int                           radius,
                                     double eps) noexcept {
    static_assert(std::is_arithmetic_v<PixelType>);

    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<float>());
    filter_result = 0.0F;

    if constexpr (std::is_same_v<PixelType, float>) {
        cv::ximgproc::guidedFilter(/*guide=*/input.data(),
                                   /*src=*/input.data(),
                                   /*dst=*/filter_result,
                                   /*radius=*/radius,
                                   /*eps=*/eps);
        return math::image<float>(std::move(filter_result));
    }

    auto conv = math::convert<float>(input);
    cv::ximgproc::guidedFilter(conv.data(), conv.data(), filter_result, radius,
                               eps);
    auto res_pic = math::image<float>(std::move(filter_result));
    return math::convert<PixelType>(res_pic);
}

/// Apply gaussian bluring to the \p input image. Uses the implementation from
/// OpenCV.
///
/// See
/// https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1
/// for a full description.
///
/// \tparam PixelType of the underlying image.
/// \param input image that shall be blurred
/// \param ksize,sigmaX,sigmaY parameters for gaussian blur.
/// If \p sigmaY is 0 the same value as \p sigmaX is used.
template <typename PixelType>
math::image<PixelType> gaussian_blur(const math::image<PixelType>& input,
                                     const cv::Size&               ksize,
                                     double                        sigmaX,
                                     double sigmaY = 0) noexcept {
    static_assert(std::is_arithmetic_v<PixelType>);

    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<PixelType>());
    cv::GaussianBlur(input.data(), filter_result, ksize, sigmaX, sigmaY);
    return math::image<PixelType>(std::move(filter_result));
}

/// Apply median bluring to the \p input image. Uses the implementation from
/// OpenCV.
///
/// See
/// https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9
/// for a full description.
///
/// \tparam PixelType of the underlying image.
/// \param input image that shall be blurred
/// \param ksize size of the square that is used as window for blurring.
template <typename PixelType>
math::image<PixelType> median_blur(const math::image<PixelType>& input,
                                   int ksize) noexcept {
    static_assert(std::is_arithmetic_v<PixelType>);

    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<PixelType>());
    cv::medianBlur(input.data(), filter_result, ksize);
    return math::image<PixelType>(std::move(filter_result));
}
}  // namespace preprocess
}  // namespace sens_loc

#endif /* end of include guard: FILTER_H_4UEYCWE1 */
