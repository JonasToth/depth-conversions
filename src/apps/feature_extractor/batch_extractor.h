#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <gsl/gsl>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace sens_loc::apps {

/// Helper class that visits a list of images and does some generic
/// processing on it.
class batch_extractor {
  public:
    batch_extractor(cv::Ptr<cv::Feature2D> detector,
                    std::string            input_pattern,
                    std::string            output_pattern)
        : _detector{std::move(detector)}
        , _input_pattern{std::move(input_pattern)}
        , _output_pattern{std::move(output_pattern)} {
        Expects(!_input_pattern.empty());
        Expects(!_output_pattern.empty());
    }

    batch_extractor(const batch_extractor&) = default;
    batch_extractor(batch_extractor&&)      = default;
    batch_extractor& operator=(const batch_extractor&) = default;
    batch_extractor& operator=(batch_extractor&&) = default;
    virtual ~batch_extractor()                    = default;

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    cv::Ptr<cv::Feature2D> _detector;
    std::string            _input_pattern;
    std::string            _output_pattern;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
