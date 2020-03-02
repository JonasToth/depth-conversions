#ifndef PRECISION_RECALL_H_G2FJDYVV
#define PRECISION_RECALL_H_G2FJDYVV

#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {

struct recognition_analysis_input {
    std::string_view                depth_image_pattern;
    std::string_view                pose_file_pattern;
    std::string_view                intrinsic_file;
    std::optional<std::string_view> mask_file;
    cv::NormTypes                   matching_norm;
    float                           keypoint_distance_threshold;

    /// Unit-Conversion for the ICP of kinect images. No other depth images
    /// are treated with ICP, so this is dirty set to a constant.
    const float unit_factor = 0.001F;
};

struct recognition_analysis_output_options {
    std::optional<std::string> backproject_pattern;
    std::optional<std::string> original_files;
    std::optional<std::string> stat_file;
    std::optional<std::string> backprojection_selected_histo;
    std::optional<std::string> relevant_histo;
    std::optional<std::string> true_positive_histo;
    std::optional<std::string> false_positive_histo;
    std::optional<std::string> true_positive_distance_histo;
    std::optional<std::string> false_positive_distance_histo;
};

int analyze_recognition_performance(
    util::processing_input                     in,
    const recognition_analysis_input&          required_data,
    const recognition_analysis_output_options& output_options);

}  // namespace sens_loc::apps

#endif /* end of include guard: PRECISION_RECALL_H_G2FJDYVV */
