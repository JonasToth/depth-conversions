#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_curvature.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc { namespace apps {
bool gauss_curv_converter::process_file(const cv::Mat &depth_image,
                                        int            idx) const noexcept {
    Expects(!_files.output.empty());

    using namespace conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    const cv::Mat gauss =
        depth_to_gaussian_curvature<double, double>(euclid_depth, intrinsic);

    bool success = cv::imwrite(fmt::format(_files.output, idx), gauss);

    return success;
}


bool mean_curv_converter::process_file(const cv::Mat &depth_image,
                                       int            idx) const noexcept {
    Expects(!_files.output.empty());

    using namespace conversion;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(depth_image, intrinsic);

    const cv::Mat mean =
        depth_to_mean_curvature<double, double>(euclid_depth, intrinsic);

    bool success = cv::imwrite(fmt::format(_files.output, idx), mean);

    return success;
}
}}  // namespace sens_loc::apps
