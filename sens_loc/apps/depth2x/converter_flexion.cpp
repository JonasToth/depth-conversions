#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc { namespace apps {

bool flexion_converter::process_file(const cv::Mat &depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());

    using namespace conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    const cv::Mat flexion =
        depth_to_flexion<double, double>(euclid_depth, intrinsic);
    const cv::Mat flexion_ushort = convert_flexion<double, ushort>(flexion);

    bool success = cv::imwrite(fmt::format(_files.output, idx), flexion_ushort);

    return success;
}

}}  // namespace sens_loc::apps
