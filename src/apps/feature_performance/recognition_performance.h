#ifndef PRECISION_RECALL_H_G2FJDYVV
#define PRECISION_RECALL_H_G2FJDYVV

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {

int analyze_recognition_performance(
    util::processing_input            in,
    std::string_view                  depth_image_pattern,
    std::string_view                  pose_file_pattern,
    std::string_view                  intrinsic_file,
    std::optional<std::string_view>   mask_file,
    cv::NormTypes                     matching_norm,
    float                             keypoint_distance_threshold,
    std::optional<std::string_view>   backproject_pattern,
    std::optional<std::string_view>   original_files,
    const std::optional<std::string>& stat_file,
    const std::optional<std::string>& backprojection_selected_histo,
    const std::optional<std::string>& relevant_histo,
    const std::optional<std::string>& true_positive_histo,
    const std::optional<std::string>& false_positive_histo);

}  // namespace sens_loc::apps

#endif /* end of include guard: PRECISION_RECALL_H_G2FJDYVV */
