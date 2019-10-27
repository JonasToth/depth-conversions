#ifndef FILTER_H_4UEYCWE1
#define FILTER_H_4UEYCWE1

#include <opencv2/imgproc.hpp>
#include <sens_loc/math/image.h>
#include <type_traits>

namespace sens_loc {

/// This namespace contains functionality to preprocess depth/range-data.
namespace preprocess {

template <typename PixelType>
math::image<float> bilateral_filter(const math::image<PixelType> &input,
                                    int distance, double sigma_color,
                                    double sigma_space) noexcept {
    cv::Mat result(input.h(), input.w(),
                   math::detail::get_opencv_type<float>());

    // The input is already in float format and does not require a prior
    // conversion.
    if constexpr (std::is_same_v<PixelType, float>) {
        cv::bilateralFilter(input.data(), result, distance, sigma_color,
                            sigma_space);
    } else {
        auto conv = math::convert<float>(input);
        cv::bilateralFilter(conv.data(), result, distance, sigma_color,
                            sigma_space);
    }
    return math::image<float>(std::move(result));
};

}  // namespace preprocess
}  // namespace sens_loc

#endif /* end of include guard: FILTER_H_4UEYCWE1 */
