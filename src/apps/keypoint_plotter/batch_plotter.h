#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <gsl/gsl>
#include <optional>
#include <sens_loc/util/correctness_util.h>
#include <string_view>

namespace sens_loc::apps {

/// \ingroup feature-plotter-driver
/// Defines a strongly typed switch for the color to use. Will be mapped
/// to the actual OpenCV-color using \c color_to_bgr.
enum class feature_color {
    green,
    blue,
    red,
    orange,
    purple,  ///< Nice purple.
    all      ///< Draws all keypoints with different colors.
};

/// Convert a color name to the corresponding \c feature_color.
/// The string must match with the enum-value, e.g. "green".
/// \ingroup feature-plotter-driver
inline feature_color str_to_color(std::string_view color_string) noexcept {
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define COLOR_SWITCH(COLOR)                                                    \
    if (color_string == #COLOR)                                                \
        return feature_color::COLOR;
    COLOR_SWITCH(all)
    COLOR_SWITCH(green)
    COLOR_SWITCH(blue)
    COLOR_SWITCH(red)
    COLOR_SWITCH(purple)
    COLOR_SWITCH(orange)

    UNREACHABLE("Unexpected color to convert");  // LCOV_EXCL_LINE
}

/// Provide a conversion from the \¢ feature_color to actual OpenCV colors.
/// \ingroup feature-plotter-driver
struct color_to_bgr {
    // Color Space in BGR.
    static cv::Scalar convert(feature_color c) {
        using cv::Scalar;
        switch (c) {
        case feature_color::all: return Scalar::all(-1);
        case feature_color::green: return Scalar(0, 255, 0);
        case feature_color::blue: return Scalar(255, 0, 0);
        case feature_color::red: return Scalar(0, 0, 255);
        case feature_color::purple: return Scalar(255, 0, 255);
        case feature_color::orange: return Scalar(45, 95, 255);
        }
        UNREACHABLE("Invalid enum-value!");  // LCOV_EXCL_LINE
    }
};

/// Helper class that visits a list of images plots keypoints onto them.
/// \ingroup feature-plotter-driver
class batch_plotter {
  public:
    batch_plotter(std::string_view                feature_file_pattern,
                  std::string_view                output_file_pattern,
                  feature_color                   color,
                  std::optional<std::string_view> target_image_file_pattern)
        : _feature_file_pattern{feature_file_pattern}
        , _ouput_file_pattern{output_file_pattern}
        , _color{color}
        , _target_image_file_pattern{target_image_file_pattern} {
        Expects(!_feature_file_pattern.empty());
        Expects(!_ouput_file_pattern.empty());
        if (_target_image_file_pattern)
            Expects(!_target_image_file_pattern->empty());
    }

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    /// Process a single feature file. Called in parallel from \c process_batch.
    [[nodiscard]] bool process_index(int idx) const noexcept;

    /// The feature-detector creates files with the keypoints. This file-pattern
    /// needs to be provided in order to plot those keypoints.
    std::string_view _feature_file_pattern;

    /// File pattern for the output image to be written to.
    /// These images are 8-bit RGB images!
    std::string_view _ouput_file_pattern;

    /// Color to draw the keypoints in.
    feature_color _color;

    /// Even though the feature-files should have a path to the original image,
    /// the features were detected on, this path might not be the desired target
    /// for plotting. This is the case for plotting multiple feature keypoints
    /// or if the path to the file is incorrect.
    std::optional<std::string_view> _target_image_file_pattern;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
