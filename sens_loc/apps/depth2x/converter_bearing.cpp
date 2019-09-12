#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>

namespace sens_loc { namespace apps {

bool bearing_converter::process_file(const cv::Mat &depth_image, int idx) const
    noexcept {
    Expects(!_files.horizontal.empty() || !_files.vertical.empty() ||
            !_files.diagonal.empty() || !_files.antidiagonal.empty());

    using namespace sens_loc::conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    bool final_result = true;
#define BEARING_PROCESS(DIRECTION)                                             \
    if (!_files.DIRECTION.empty()) {                                           \
        cv::Mat bearing =                                                      \
            depth_to_bearing<direction::DIRECTION, double, double>(            \
                euclid_depth, intrinsic);                                      \
        bool success = cv::imwrite(fmt::format(_files.DIRECTION, idx),         \
                                   convert_bearing<double, ushort>(bearing));  \
        final_result &= success;                                               \
    }

    BEARING_PROCESS(horizontal)
    BEARING_PROCESS(vertical)
    BEARING_PROCESS(diagonal)
    BEARING_PROCESS(antidiagonal)

#undef BEARING_PROCESS

    return final_result;
}

}}  // namespace sens_loc::apps
