#ifndef KEYPOINT_DISTRIBUTION_H_K2G0XHSJ
#define KEYPOINT_DISTRIBUTION_H_K2G0XHSJ

#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {

int analyze_keypoint_distribution(
    util::processing_input            in,
    unsigned int                      image_width,
    unsigned int                      image_height,
    const std::optional<std::string>& stat_file,
    const std::optional<std::string>& response_histo,
    const std::optional<std::string>& size_histo,
    const std::optional<std::string>& kp_distance_histo,
    const std::optional<std::string>& kp_distribution_histo);
}  // namespace sens_loc::apps

#endif /* end of include guard: KEYPOINT_DISTRIBUTION_H_K2G0XHSJ */
