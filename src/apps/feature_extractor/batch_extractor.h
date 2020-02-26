#ifndef BATCH_EXTRACTOR_H_IVH3CLLQ
#define BATCH_EXTRACTOR_H_IVH3CLLQ

#include <algorithm>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <optional>
#include <sens_loc/util/correctness_util.h>

using namespace std;
using namespace cv;

namespace sens_loc::apps {

/// Helper class that visits a list of images and extracts features with the
/// provided detectors.
/// \ingroup feature-extractor-driver
class batch_extractor {
  public:
    using filter_func = function<vector<KeyPoint>::iterator(vector<KeyPoint>&)>;

    /// \param detector,descriptor Algorithms used for detection and
    /// description.
    /// \param input_pattern,output_pattern Formattable string for IO.
    /// \param keypoint_filter Callable that determines if a keypoint shall be
    /// dropped from consideration. All keypoints with 'keypoint_filter(kp) ==
    /// true' are removed.
    batch_extractor(Ptr<Feature2D>      detector,
                    Ptr<Feature2D>      descriptor,
                    string_view         input_pattern,
                    string_view         output_pattern,
                    vector<filter_func> keypoint_filter)
        : _detector{move(detector)}
        , _descriptor{move(descriptor)}
        , _input_pattern{input_pattern}
        , _ouput_pattern{output_pattern}
        , _keypoint_filter{move(keypoint_filter)} {
        Expects(!_input_pattern.empty());
        Expects(!_ouput_pattern.empty());
        Expects(!_detector.empty());
    }

    /// Process a whole batch of files in the range [start, end].
    [[nodiscard]] bool process_batch(int start, int end) const noexcept;

  private:
    /// Detect and describe one single index. Handles the IO as well.
    [[nodiscard]] bool process_index(int idx) const noexcept;

    /// Do IO and handle detection down to \c compute_features.
    bool process_detector(const math::image<uchar>& image,
                          const string&             out_file,
                          const string&             in_file) const noexcept;

    /// Compute and filter keypoints and run the descriptor on them
    /// afterwards.
    [[nodiscard]] pair<vector<KeyPoint>, Mat>
    compute_features(const math::image<uchar>& img) const noexcept;


    mutable Ptr<Feature2D> _detector;
    mutable Ptr<Feature2D> _descriptor;
    string_view            _input_pattern;
    string_view            _ouput_pattern;
    vector<filter_func>    _keypoint_filter;
};
}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_EXTRACTOR_H_IVH3CLLQ */
