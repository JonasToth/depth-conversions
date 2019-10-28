#ifndef FILTER_H_4UEYCWE1
#define FILTER_H_4UEYCWE1

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sens_loc/math/image.h>
#include <type_traits>

namespace sens_loc {

/// This namespace contains functionality to preprocess depth/range-data.
/// It wraps functionality found in OpenCV (-contrib) and makes it accessable
/// for the strongly typed interfaces.
namespace preprocess {

template <typename PixelType>
math::image<PixelType> bilateral_filter(const math::image<PixelType> &input,
                                        int distance, double sigma_color,
                                        double sigma_space) noexcept {
    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<float>());
    filter_result = 0.0f;

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


template <typename PixelType>
math::image<float> guided_filter(const math::image<PixelType> &input,
                                 int radius, double eps) noexcept {
    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<float>());
    filter_result = 0.0f;

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

template <typename PixelType>
math::image<PixelType> gaussian_blur(const math::image<PixelType> &input,
                                     cv::Size ksize, double sigmaX,
                                     double sigmaY = 0) noexcept {
    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<PixelType>());
    cv::GaussianBlur(input.data(), filter_result, ksize, sigmaX, sigmaY);
    return math::image<PixelType>(std::move(filter_result));
}

template <typename PixelType>
math::image<PixelType> median_blur(const math::image<PixelType> &input,
                                   int ksize) noexcept {
    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<PixelType>());
    cv::medianBlur(input.data(), filter_result, ksize);
    return math::image<PixelType>(std::move(filter_result));
}

template <typename PixelType>
math::image<PixelType> blur(const math::image<PixelType> &input,
                            cv::Size                      ksize) noexcept {
    cv::Mat filter_result(input.h(), input.w(),
                          math::detail::get_opencv_type<PixelType>());
    cv::blur(input.data(), filter_result, ksize);
    return math::image<PixelType>(std::move(filter_result));
}

template <typename PixelType>
math::image<uchar> edge_preserving_filter(const math::image<PixelType> &input,
                                          int size, double threshold) noexcept {
    cv::Mat scaled = input.data() * (1. / 255.);
    cv::Mat narrowed;
    scaled.convertTo(narrowed, CV_8U);

    cv::Mat converted;
    cv::cvtColor(narrowed, converted, cv::COLOR_GRAY2BGR, 3);

    cv::Mat filter_result(input.h(), input.w(), CV_8UC3);
    cv::ximgproc::edgePreservingFilter(converted, filter_result, size,
                                       threshold);

    cv::Mat channels[3];
    // The actual splitting.
    cv::split(filter_result, channels);

    return math::image<uchar>(channels[0]);
}

template <typename PixelType>
math::image<uchar> anistropic_diffusion(const math::image<PixelType> &input,
                                        float alpha, float K,
                                        int iterations) noexcept {
    cv::Mat scaled = input.data() * (1. / 255.);
    cv::Mat narrowed;
    scaled.convertTo(narrowed, CV_8U);

    cv::Mat converted;
    cv::cvtColor(narrowed, converted, cv::COLOR_GRAY2BGR, 3);

    cv::Mat filter_result(input.h(), input.w(), CV_8UC3);
    cv::ximgproc::anisotropicDiffusion(converted, filter_result, alpha, K,
                                       iterations);

    cv::Mat channels[3];
    // The actual splitting.
    cv::split(filter_result, channels);

    return math::image<uchar>(channels[0]);
}

}  // namespace preprocess
}  // namespace sens_loc

#endif /* end of include guard: FILTER_H_4UEYCWE1 */
