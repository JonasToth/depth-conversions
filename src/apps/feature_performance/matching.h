#ifndef MATCHING_H_HSZOIMBW
#define MATCHING_H_HSZOIMBW

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {
int analyze_matching(util::processing_input            in,
                     cv::NormTypes                     norm_to_use,
                     bool                              crosscheck,
                     const std::optional<std::string>& stat_file,
                     const std::optional<std::string>& matched_distance_histo,
                     const std::optional<std::string_view>& output_pattern,
                     const std::optional<std::string_view>& original_files);
}  // namespace sens_loc::apps

#endif /* end of include guard: MATCHING_H_HSZOIMBW */
