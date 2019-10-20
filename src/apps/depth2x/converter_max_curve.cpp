#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_max_curve.h>

namespace sens_loc::apps {

bool max_curve_converter::process_file(math::image<double> depth_image,
                                       int                 idx) const noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const auto max_curve =
        depth_to_max_curve<double, double>(depth_image, intrinsic);
    bool success =
        cv::imwrite(fmt::format(_files.output, idx),
                    convert_max_curve<double, ushort>(max_curve).data());

    return success;
}
}  // namespace sens_loc::apps
