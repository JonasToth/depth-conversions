#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>

namespace sens_loc::apps {

bool bearing_converter::process_file(math::image<double> depth_image,
                                     int                 idx) const noexcept {
    Expects(!_files.horizontal.empty() || !_files.vertical.empty() ||
            !_files.diagonal.empty() || !_files.antidiagonal.empty());
    using namespace sens_loc::conversion;

    bool final_result = true;
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BEARING_PROCESS(DIRECTION)                                             \
    if (!_files.DIRECTION.empty()) {                                           \
        math::image<double> bearing =                                          \
            depth_to_bearing<direction::DIRECTION, double, double>(            \
                depth_image, intrinsic);                                       \
        bool success =                                                         \
            cv::imwrite(fmt::format(_files.DIRECTION, idx),                    \
                        convert_bearing<double, ushort>(bearing).data());      \
        final_result &= success;                                               \
    }

    BEARING_PROCESS(horizontal)
    BEARING_PROCESS(vertical)
    BEARING_PROCESS(diagonal)
    BEARING_PROCESS(antidiagonal)

#undef BEARING_PROCESS

    return final_result;
}

}  // namespace sens_loc::apps
