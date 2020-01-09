#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <algorithm>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sens_loc/util/correctness_util.h>

namespace sens_loc::apps {

/// Small wrapper-class that encodes the information for detector-configuration
/// and file-IO configuration in batch processing.
/// \ingroup feature-extractor-driver
struct Detector {
    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::Feature2D> descriptor;
    std::string_view       output_pattern;

    Detector(cv::Ptr<cv::Feature2D> detecta_and_descripta,
             std::string_view       output_pattern)
        : detector{std::move(detecta_and_descripta)}
        , descriptor{nullptr}
        , output_pattern{output_pattern} {
        Expects(!detector.empty());
        Expects(descriptor.empty());
    }

    Detector(cv::Ptr<cv::Feature2D> detecta,
             cv::Ptr<cv::Feature2D> descripta,
             std::string_view       output_pattern)
        : detector{std::move(detecta)}
        , descriptor{std::move(descripta)}
        , output_pattern{output_pattern} {
        Expects(!detector.empty());
        Expects(!descriptor.empty());
    }
};

/// Helper class that visits a list of images and does some generic
/// processing on it.
/// \ingroup feature-extractor-driver
class batch_extractor {
  public:
    batch_extractor(const std::vector<Detector>& detectors,
                    std::string_view             input_pattern)
        : _detectors{detectors}
        , _input_pattern{input_pattern} {
        Expects(!_input_pattern.empty());
        Expects(std::none_of(
            std::begin(detectors), std::end(detectors), [](const Detector& d) {
                return d.output_pattern.empty() || d.detector.empty();
            }));
    }

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    const std::vector<Detector>& _detectors;
    std::string_view             _input_pattern;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
