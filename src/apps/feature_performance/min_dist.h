#ifndef MIN_DIST_H_AHV2P7Y1
#define MIN_DIST_H_AHV2P7Y1

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>

namespace sens_loc::apps {
int analyze_min_distance(std::string_view                  input_pattern,
                         int                               start_idx,
                         int                               end_idx,
                         cv::NormTypes                     norm_to_use,
                         const std::optional<std::string>& stat_file);
}  // namespace sens_loc::apps

#endif /* end of include guard: MIN_DIST_H_AHV2P7Y1 */
