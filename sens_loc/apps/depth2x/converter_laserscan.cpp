#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc { namespace apps {
bool range_converter::process_file(cv::Mat depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    /// The input 'depth_image' is already in range-form as its beeing
    /// preprocessed.
    cv::Mat depth_16bit(depth_image.rows, depth_image.cols, CV_16U);
    depth_image.convertTo(depth_16bit, CV_16U);
    bool success = cv::imwrite(fmt::format(_files.output, idx), depth_16bit);

    return success;
}
}}  // namespace sens_loc::apps
