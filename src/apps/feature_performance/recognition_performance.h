#ifndef PRECISION_RECALL_H_G2FJDYVV
#define PRECISION_RECALL_H_G2FJDYVV

#include <gsl/gsl>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string_view>
#include <util/common_structures.h>

namespace sens_loc::apps {

/// Defines the color and line strength used for backprojection.
struct backproject_style {
    backproject_style() = default;
    backproject_style(int r, int g, int b, int strength)
        : color{CV_RGB(r, g, b)}
        , strength{strength} {
        Expects(strength > 0);
        Expects(r >= 0);
        Expects(g >= 0);
        Expects(b >= 0);
        Expects(r < 256);
        Expects(g < 256);
        Expects(b < 256);
    }

    cv::Scalar color    = CV_RGB(0, 0, 0);
    int        strength = 1;
};

/// Composition of styles on how the backprojections are plotted.
/// \sa backproject_style
struct backproject_config {
    backproject_style true_positive;
    backproject_style false_negative;
    backproject_style false_positive;
};

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
    const recognition_analysis_output_options& output_options,
    const backproject_config&                  backproject_config);

}  // namespace sens_loc::apps

#endif /* end of include guard: PRECISION_RECALL_H_G2FJDYVV */
