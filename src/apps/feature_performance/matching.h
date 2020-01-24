#ifndef MATCHING_H_HSZOIMBW
#define MATCHING_H_HSZOIMBW

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>

namespace sens_loc::apps {
int analyze_matching(
    std::string_view                input_pattern,
    int                             start_idx,
    int                             end_idx,
    cv::NormTypes                   norm_to_use,
    bool                            crosscheck,
    std::optional<std::string_view> output_pattern = std::nullopt,
    std::optional<std::string_view> original_files = std::nullopt);
}  // namespace sens_loc::apps

#endif /* end of include guard: MATCHING_H_HSZOIMBW */
