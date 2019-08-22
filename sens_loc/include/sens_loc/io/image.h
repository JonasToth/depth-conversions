#ifndef IMAGE_H_WIIAQPH0
#define IMAGE_H_WIIAQPH0

#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <utility>

namespace sens_loc { namespace image {
/// Perfect forward loading call to opencv.
/// Transform the result into a `std::optional<cv::Mat>` for better error
/// handling.
template <typename... Arg>
std::optional<cv::Mat> load(Arg &&... args) {
    cv::Mat result = cv::imread(std::forward<Arg>(args)...);
    if (result.data == nullptr)
        return std::nullopt;
    return {result};
}
}}  // namespace sens_loc::image

#endif /* end of include guard: IMAGE_H_WIIAQPH0 */
