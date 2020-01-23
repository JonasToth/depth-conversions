#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <algorithm>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sens_loc/util/correctness_util.h>

namespace sens_loc::apps {

/// Helper class that visits a list of images and extracts features with the
/// provided detectors.
/// \ingroup feature-extractor-driver
class batch_extractor {
  public:
    batch_extractor(cv::Ptr<cv::Feature2D> detector,
                    cv::Ptr<cv::Feature2D> descriptor,
                    std::string_view       input_pattern,
                    std::string_view       output_pattern)
        : _detector{std::move(detector)}
        , _descriptor{std::move(descriptor)}
        , _input_pattern{input_pattern}
        , _ouput_pattern{output_pattern} {
        Expects(!_input_pattern.empty());
        Expects(!_ouput_pattern.empty());
        Expects(!_detector.empty());
        Expects(!_descriptor.empty());
    }

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    [[nodiscard]] bool process_index(int idx) const noexcept;

    mutable cv::Ptr<cv::Feature2D> _detector;
    mutable cv::Ptr<cv::Feature2D> _descriptor;
    std::string_view       _input_pattern;
    std::string_view       _ouput_pattern;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
