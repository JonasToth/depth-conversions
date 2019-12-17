#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sens_loc/util/correctness_util.h>

namespace sens_loc::apps {

/// \ingroup feature-extractor-driver
enum class feature_color { green, blue, red, orange, purple, all };

/// \ingroup feature-extractor-driver
struct color_to_rgb {
    static cv::Scalar convert(feature_color c) {
        using cv::Scalar;
        switch (c) {
        case feature_color::green: return Scalar(0, 255, 0);
        case feature_color::blue: return Scalar(255, 0, 0);
        case feature_color::red: return Scalar(0, 0, 255);
        case feature_color::orange: return Scalar(0, 255, 255);
        case feature_color::purple: return Scalar(255, 0, 255);
        case feature_color::all: return Scalar::all(-1);
        }
        UNREACHABLE("Invalid enum-value!");  // LCOV_EXCL_LINE
    }
};

/// Small wrapper-class that encodes the information for detector-configuration
/// and file-IO configuration in batch processing.
/// \ingroup feature-extractor-driver
struct Detector {
    cv::Ptr<cv::Feature2D> detector;
    std::string            output_pattern;
    feature_color          color = feature_color::all;
};

/// Helper class that visits a list of images and does some generic
/// processing on it.
/// \ingroup feature-extractor-driver
class batch_extractor {
  public:
    batch_extractor(Detector detector, std::string input_pattern)
        : _detector{std::move(detector)}
        , _input_pattern{std::move(input_pattern)} {
        Expects(!_input_pattern.empty());
        Expects(!_detector.output_pattern.empty());
    }

    batch_extractor(const batch_extractor&) = default;
    batch_extractor(batch_extractor&&)      = default;
    batch_extractor& operator=(const batch_extractor&) = default;
    batch_extractor& operator=(batch_extractor&&) = default;
    virtual ~batch_extractor()                    = default;

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    Detector    _detector;
    std::string _input_pattern;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
