#include "batch_filter.h"

#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/preprocess/filter.h>

namespace sens_loc { namespace apps {

bool batch_filter::process_file(math::image<double> depth_image, int idx) const
    noexcept {
    auto as_float = math::convert<float>(depth_image);

    for (auto&& op : _operations)
        as_float = op->filter(as_float);

    const bool success = cv::imwrite(fmt::format(_files.output, idx),
                                     math::convert<ushort>(as_float).data());
    return success;
}

}}  // namespace sens_loc::apps
