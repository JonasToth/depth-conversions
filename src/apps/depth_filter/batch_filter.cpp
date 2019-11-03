#include "batch_filter.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/preprocess/filter.h>

namespace sens_loc::apps {

bool batch_filter::process_file(const math::image<float>& depth_image,
                                int                       idx) const noexcept {
    // Thats a NO-OP because the type already matches, 'convert' short
    // circuits that.
    math::image<float> result = math::convert<float>(depth_image);

    std::for_each(std::begin(_operations), std::end(_operations),
                  [&](auto&& op) { result = op->filter(result); });

    const bool success = cv::imwrite(fmt::format(_files.output, idx),
                                     math::convert<ushort>(result).data());
    return success;
}

}  // namespace sens_loc::apps
