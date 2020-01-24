#ifndef IO_H_FM0VH364
#define IO_H_FM0VH364

#include <optional>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>
#include <string>

namespace sens_loc::apps {

/// Expects to load a 16bit grayscale image.
/// It converts those images to 8bit grayscale images that will be processed.
inline std::optional<math::image<uchar>>
load_file(const std::string& name) noexcept {
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(name, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return {};

    return math::convert<uchar>(
        math::image<ushort>(depth_image->data() / 255.));  // NOLINT
}
}  // namespace sens_loc::apps

#endif /* end of include guard: IO_H_FM0VH364 */
