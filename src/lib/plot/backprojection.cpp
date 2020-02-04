#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/plot/backprojection.h>

using namespace cv;
using namespace std;

static bool invalid_keypoint(const Point2f& p, int width, int height) noexcept {
    return p.x < 0.0F || p.y < 0.0F || p.x > gsl::narrow<float>(width) ||
           p.y > gsl::narrow_cast<float>(height);
}

static void draw_line_correspondence(Mat&                    underground,
                                     const vector<KeyPoint>& set1,
                                     const vector<KeyPoint>& set2,
                                     const Scalar&           line_color,
                                     int                     line_thickness,
                                     int                     img_width,
                                     int img_height) noexcept {
    Expects(set1.size() == set2.size());

    for (std::size_t i = 0UL; i < set1.size(); ++i) {
        if (invalid_keypoint(set2[i].pt, img_width, img_height))
            continue;

        line(underground, set1[i].pt, set2[i].pt,
             /*color=*/line_color,
             /*thickness=*/line_thickness,
             /*linetype=*/LINE_AA);
    }
}

namespace sens_loc::plot {

cv::Mat
backprojection_correspondence(const cv::Mat&             feature_file,
                              const math::imagepoints_t& keypoints_this,
                              const math::imagepoints_t& keypoints_other,
                              const cv::Scalar&          line_color,
                              const int strength_relevant) noexcept {

    Expects(keypoints_this.size() == keypoints_other.size());
    Expects(!feature_file.empty());

    cv::Mat underground;
    if (feature_file.channels() == 1) {
        Expects(feature_file.type() == CV_8U);
        cvtColor(feature_file, underground, COLOR_GRAY2BGR);
    } else {
        Expects(feature_file.channels() == 3);
        Expects(feature_file.type() == CV_8UC3);
        underground = feature_file;
    }

    using camera_models::coords_to_keypoint;
    vector<KeyPoint> kps_in_this    = coords_to_keypoint(keypoints_this);
    vector<KeyPoint> kps_from_other = coords_to_keypoint(keypoints_other);
    Ensures(kps_in_this.size() == kps_from_other.size());

    // Draw keypoints as circles.
    draw_line_correspondence(underground, kps_in_this, kps_from_other,
                             line_color, strength_relevant, feature_file.cols,
                             feature_file.rows);
    return underground;
}
}  // namespace sens_loc::plot
