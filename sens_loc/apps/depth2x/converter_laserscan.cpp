#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc { namespace apps {
bool range_converter::process_file(const cv::Mat &depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());

    using namespace conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    cv::Mat depth_converted(depth_image.rows, depth_image.cols,
                            depth_image.type());
    euclid_depth.convertTo(depth_converted, depth_image.type());

    bool success =
        cv::imwrite(fmt::format(_files.output, idx), depth_converted);

    return success;
}
}}  // namespace sens_loc::apps
