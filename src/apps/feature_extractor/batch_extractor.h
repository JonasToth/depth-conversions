#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <algorithm>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <optional>
#include <sens_loc/util/correctness_util.h>

namespace sens_loc::apps {

/// Helper class that visits a list of images and extracts features with the
/// provided detectors.
/// \ingroup feature-extractor-driver
class batch_extractor {
  public:
    using filter_func = std::function<bool(const cv::KeyPoint&)>;

    /// \param detector,descriptor Algorithms used for detection and
    /// description.
    /// \param input_pattern,output_pattern Formattable string for IO.
    /// \param keypoint_filter Callable that determines if a keypoint shall be
    /// dropped from consideration. All keypoints with 'keypoint_filter(kp) ==
    /// true' are removed.
    batch_extractor(cv::Ptr<cv::Feature2D>     detector,
                    cv::Ptr<cv::Feature2D>     descriptor,
                    std::string_view           input_pattern,
                    std::string_view           output_pattern,
                    std::optional<filter_func> keypoint_filter = std::nullopt)
        : _detector{std::move(detector)}
        , _descriptor{std::move(descriptor)}
        , _input_pattern{input_pattern}
        , _ouput_pattern{output_pattern}
        , _keypoint_filter{std::move(keypoint_filter)} {
        Expects(!_input_pattern.empty());
        Expects(!_ouput_pattern.empty());
        Expects(!_detector.empty());
        Expects(!_descriptor.empty());
    }

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    /// Detect and describe one single index. Handles the IO as well.
    [[nodiscard]] bool process_index(int idx) const noexcept;

    /// Do IO and handle detection down to \c compute_features.
    bool process_detector(const math::image<uchar>& image,
                          const std::string&        out_file,
                          const std::string&        in_file) const noexcept;

    /// Compute and filter keypoints and run the descriptor on them
    /// afterwards.
    [[nodiscard]] std::pair<std::vector<cv::KeyPoint>, cv::Mat>
    compute_features(const math::image<uchar>& img) const noexcept;


    mutable cv::Ptr<cv::Feature2D> _detector;
    mutable cv::Ptr<cv::Feature2D> _descriptor;
    std::string_view               _input_pattern;
    std::string_view               _ouput_pattern;
    std::optional<filter_func>     _keypoint_filter;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
