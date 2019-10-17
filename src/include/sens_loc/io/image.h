#ifndef IMAGE_H_WIIAQPH0
#define IMAGE_H_WIIAQPH0

#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <utility>

namespace sens_loc {

/// This namespace contains all functions for file io and helper for data
/// loading and storing.
namespace io {
/// Perfect forward loading call to opencv's \c cv::imread.
///
/// \returns \c std::optional<cv::Mat> for better error handling.
/// If the optional contains a value, the load was successful, otherwise it is
/// \c None.
template <typename... Arg>
std::optional<cv::Mat> load_image(Arg &&... args) {
    cv::Mat result = cv::imread(std::forward<Arg>(args)...);
    if (result.data == nullptr)
        return std::nullopt;
    return {result};
}
}}  // namespace sens_loc::io

#endif /* end of include guard: IMAGE_H_WIIAQPH0 */
