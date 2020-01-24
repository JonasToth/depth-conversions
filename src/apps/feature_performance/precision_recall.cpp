#include "precision_recall.h"

#include "gsl/gsl_assert"

#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/io/intrinsics.h>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

namespace {
class prec_recall_analysis {
  public:
    prec_recall_analysis(std::string_view feature_file_pattern,
                         std::string_view depth_image_pattern,
                         std::string_view pose_file_pattern,
                         std::string_view intrinsic_file,
                         cv::NormTypes    matching_norm) noexcept
        : _feature_file_pattern{feature_file_pattern}
        , _depth_image_pattern{depth_image_pattern}
        , _pose_file_pattern{pose_file_pattern}
        , _matching_norm{matching_norm} {
        Expects(!_feature_file_pattern.empty());
        Expects(!_depth_image_pattern.empty());
        Expects(!_pose_file_pattern.empty());
        Expects(!_intrinsic_file.empty());

        std::ifstream intrinsic{std::string(intrinsic_file)};
        _intrinsic = sens_loc::io<float, sens_loc::camera_models::pinhole>::load_intrinsic(
            intrinsic);
    }

    void operator()(int                                      idx,
                    std::optional<std::vector<cv::KeyPoint>> keypoints,
                    std::optional<cv::Mat> descriptors) noexcept {
        Expects(keypoints);
        Expects(!keypoints->empty());
        Expects(descriptors);
        Expects(std::size_t(descriptors->rows) == keypoints->size());

        const int previous_idx = idx - 1;

        using namespace sens_loc::apps;
        using namespace std;
        using namespace cv;

        const FileStorage previous_feature_file =
            open_feature_file(_feature_file_pattern, previous_idx);
        vector<KeyPoint> previous_keypoints =
            load_keypoints(previous_feature_file);
        Mat previous_descriptors = load_descriptors(previous_feature_file);
    }

    void postprocess() noexcept {}

  private:
    std::string_view                        _feature_file_pattern;
    std::string_view                        _depth_image_pattern;
    std::string_view                        _pose_file_pattern;
    sens_loc::camera_models::pinhole<float> _intrinsic;
    cv::NormTypes                           _matching_norm;
};
}  // namespace

namespace sens_loc::apps {
int analyze_precision_recall(std::string_view feature_file_pattern,
                             int              start_idx,
                             int              end_idx,
                             std::string_view depth_image_pattern,
                             std::string_view pose_file_pattern,
                             std::string_view intrinsic_file,
                             cv::NormTypes    matching_norm) {
    Expects(start_idx < end_idx &&
            "Precision-Recall calculation requires at least two images");

    using visitor =
        statistic_visitor<prec_recall_analysis, required_data::keypoints |
                                                    required_data::descriptors>;
    auto analysis_v =
        visitor{feature_file_pattern, feature_file_pattern, depth_image_pattern,
                pose_file_pattern,    intrinsic_file,       matching_norm};

    // Consecutive images are matched and analysed, therefore the first index
    // must be skipped.
    auto f = parallel_visitation(start_idx + 1, end_idx, analysis_v);
    f.postprocess();

    return 0;
}

}  // namespace sens_loc::apps
