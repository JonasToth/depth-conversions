#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_max_curve.h>

namespace sens_loc { namespace apps {

bool max_curve_converter::process_file(const cv::Mat &depth_image,
                                       int            idx) const noexcept {
    Expects(!_files.output.empty());

    using namespace conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    const cv::Mat max_curve =
        depth_to_max_curve<double, double>(euclid_depth, intrinsic);
    const cv::Mat curve_ushort = convert_max_curve<double, ushort>(max_curve);

    bool success = cv::imwrite(fmt::format(_files.output, idx), curve_ushort);

    return success;
}
}}  // namespace sens_loc::apps
