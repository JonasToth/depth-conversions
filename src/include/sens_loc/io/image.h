#ifndef IMAGE_H_WIIAQPH0
#define IMAGE_H_WIIAQPH0

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sens_loc/math/image.h>
#include <type_traits>
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
std::optional<math::image<PixelType>> load_image(Arg&&... args) {
    static_assert(std::is_arithmetic_v<PixelType>);

    cv::Mat result = cv::imread(std::forward<Arg>(args)...);
    if (result.data == nullptr)
        return std::nullopt;
    if (result.type() != math::detail::get_opencv_type<PixelType>())
        return std::nullopt;
    return {math::image<PixelType>(std::move(result))};
}

/// Expects to load a 16bit grayscale image.
/// It converts those images to 8bit grayscale images that will be processed.
inline std::optional<math::image<uchar>>
load_as_8bit_gray(const std::string& name) noexcept {
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(name, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return {};

    return math::convert<uchar>(
        math::image<ushort>(depth_image->data() / 255.));  // NOLINT
}

}  // namespace io
}  // namespace sens_loc

#endif /* end of include guard: IMAGE_H_WIIAQPH0 */
