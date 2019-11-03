#include "converter_scale.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_scaling.h>

namespace sens_loc::apps {

bool scale_converter::process_file(const math::image<double>& depth_image,
                                   int idx) const noexcept {
    Expects(!_files.output.empty());
    using namespace sens_loc::conversion;
    const auto res = depth_scaling(depth_image, _scale, _offset);
    cv::Mat    depth_16bit(depth_image.h(), depth_image.w(), CV_16U);
    res.data().convertTo(depth_16bit, CV_16U);
    bool success = cv::imwrite(fmt::format(_files.output, idx), depth_16bit);

    return success;
}

}  // namespace sens_loc::apps
