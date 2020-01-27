#ifndef KEYPOINT_DISTRIBUTION_H_K2G0XHSJ
#define KEYPOINT_DISTRIBUTION_H_K2G0XHSJ

#include <string_view>

namespace sens_loc::apps {

int analyze_keypoint_distribution(std::string_view input_pattern,
                                  int              start_idx,
                                  int              end_idx,
                                  unsigned int     image_width,
                                  unsigned int     image_height);
}  // namespace sens_loc::apps

#endif /* end of include guard: KEYPOINT_DISTRIBUTION_H_K2G0XHSJ */
