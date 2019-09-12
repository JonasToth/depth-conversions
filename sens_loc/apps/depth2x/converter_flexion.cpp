#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc::apps {
bool flexion_converter::process_file(cv::Mat depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const cv::Mat flexion =
        depth_to_flexion<double, double>(depth_image, intrinsic);
    const cv::Mat flexion_ushort = convert_flexion<double, ushort>(flexion);
    bool success = cv::imwrite(fmt::format(_files.output, idx), flexion_ushort);

    return success;
}

}  // namespace sens_loc::apps
