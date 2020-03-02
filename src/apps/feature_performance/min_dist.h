#ifndef MIN_DIST_H_AHV2P7Y1
#define MIN_DIST_H_AHV2P7Y1

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {
int analyze_min_distance(util::processing_input            in,
                         cv::NormTypes                     norm_to_use,
                         const std::optional<std::string>& stat_file,
                         const std::optional<std::string>& min_dist_histo);
}  // namespace sens_loc::apps

#endif /* end of include guard: MIN_DIST_H_AHV2P7Y1 */
