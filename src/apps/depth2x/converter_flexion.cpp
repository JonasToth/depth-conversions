#include "converters.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>


namespace sens_loc::apps {
bool flexion_converter::process_file(math::image<double> depth_image,
                                     int                 idx) const noexcept {
    Expects(!_files.output.empty());
    using namespace conversion;

    const auto flexion =
        depth_to_flexion<double, double>(depth_image, intrinsic);
    ;
    bool success = cv::imwrite(fmt::format(_files.output, idx),
                               convert_flexion<double, ushort>(flexion).data());

    return success;
}

}  // namespace sens_loc::apps
