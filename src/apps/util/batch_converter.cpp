#include "batch_converter.h"

#include "parallel_processing.h"

#include <fmt/core.h>
#include <gsl/gsl>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>

namespace sens_loc::apps {

bool batch_converter::process_index(int idx) const noexcept {
    Expects(!_files.input.empty());
    const std::string input_file = fmt::format(_files.input, idx);
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(input_file, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return false;

    std::optional<math::image<float>> pp_image =
        this->preprocess_depth(*depth_image);

    if (!pp_image)
        return false;

    return this->process_file(*pp_image, idx);
}

std::optional<math::image<float>>
batch_converter::preprocess_depth(const math::image<ushort>& depth_image) const
    noexcept {
    return math::convert<float>(depth_image);
}

bool batch_converter::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(
        start, end,
        [this](int idx) noexcept -> bool { return this->process_index(idx); });
}

}  // namespace sens_loc::apps
