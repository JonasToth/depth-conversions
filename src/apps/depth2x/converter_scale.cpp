#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_scaling.h>

namespace sens_loc::apps {

bool scale_converter::process_file(cv::Mat depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());
    using namespace sens_loc::conversion;
    const cv::Mat res = depth_scaling(depth_image, _scale, _offset);
    bool success = cv::imwrite(fmt::format(_files.output, idx), res);

    return success;
}

}  // namespace sens_loc::apps
