#ifndef IMAGE_H_WIIAQPH0
#define IMAGE_H_WIIAQPH0

#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <sens_loc/math/image.h>
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
template <typename PixelType, typename... Arg>
std::optional<math::image<PixelType>> load_image(Arg &&... args) {
    cv::Mat result = cv::imread(std::forward<Arg>(args)...);
    if (result.data == nullptr)
        return std::nullopt;
    if (result.type() != math::detail::get_opencv_type<PixelType>())
        return std::nullopt;
    return {math::image<PixelType>(std::move(result))};
}
}  // namespace io
}  // namespace sens_loc

#endif /* end of include guard: IMAGE_H_WIIAQPH0 */
