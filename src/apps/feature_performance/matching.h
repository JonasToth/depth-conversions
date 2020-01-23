#ifndef MATCHING_H_HSZOIMBW
#define MATCHING_H_HSZOIMBW

#include <opencv2/core/base.hpp>
#include <string_view>

namespace sens_loc::apps {
int analyze_matching(std::string_view input_pattern,
                     int              start_idx,
                     int              end_idx,
                     cv::NormTypes    norm_to_use,
                     bool             crosscheck);
}  // namespace sens_loc::apps

#endif /* end of include guard: MATCHING_H_HSZOIMBW */
