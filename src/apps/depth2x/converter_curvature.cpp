#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_curvature.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc::apps {
bool gauss_curv_converter::process_file(cv::Mat depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const cv::Mat gauss =
        depth_to_gaussian_curvature<double, double>(depth_image, intrinsic);
    const auto converted = conversion::curvature_to_image<double, ushort>(
        gauss, depth_image, -10., 10.);
    bool success = cv::imwrite(fmt::format(_files.output, idx), converted);

    return success;
}


bool mean_curv_converter::process_file(cv::Mat depth_image, int idx) const
    noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const cv::Mat mean =
        depth_to_mean_curvature<double, double>(depth_image, intrinsic);
    const auto converted = conversion::curvature_to_image<double, ushort>(
        mean, depth_image, -20., 20.);
    bool success = cv::imwrite(fmt::format(_files.output, idx), converted);

    return success;
}
}  // namespace sens_loc::apps
