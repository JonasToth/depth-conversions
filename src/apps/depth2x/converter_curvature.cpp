#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_curvature.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc::apps {
bool gauss_curv_converter::process_file(math::image<double> depth_image,
                                        int idx) const noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const auto gauss =
        depth_to_gaussian_curvature<double, double>(depth_image, intrinsic);
    const auto converted = conversion::curvature_to_image<double, ushort, double>(
        gauss, depth_image, lower_bound, upper_bound);
    bool success =
        cv::imwrite(fmt::format(_files.output, idx), converted.data());

    return success;
}


bool mean_curv_converter::process_file(math::image<double> depth_image,
                                       int                 idx) const noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const auto mean =
        depth_to_mean_curvature<double, double>(depth_image, intrinsic);
    const auto converted = conversion::curvature_to_image<double, ushort, double>(
        mean, depth_image, lower_bound, upper_bound);
    bool success =
        cv::imwrite(fmt::format(_files.output, idx), converted.data());

    return success;
}
}  // namespace sens_loc::apps
