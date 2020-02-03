#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/plot/backprojection.h>

static bool
invalid_keypoint(const cv::Point2f& p, int width, int height) noexcept {
    return p.x < 0.0F || p.y < 0.0F || p.x > gsl::narrow<float>(width) ||
           p.y > gsl::narrow_cast<float>(height);
}

namespace sens_loc::plot {

cv::Mat backprojection_correspondence(
    const math::image<uchar>&  feature_file,
    const math::imagepoints_t& keypoints_this,
    const math::imagepoints_t& keypoints_other) noexcept {
    using namespace cv;
    using namespace std;

    Expects(keypoints_this.size() == keypoints_other.size());
    Expects(!feature_file.data().empty());

    cv::Mat underground;
    cvtColor(feature_file.data(), underground, COLOR_GRAY2BGR);

    using camera_models::coords_to_keypoint;
    vector<KeyPoint> kps_in_this    = coords_to_keypoint(keypoints_this);
    vector<KeyPoint> kps_from_other = coords_to_keypoint(keypoints_other);
    Ensures(kps_in_this.size() == kps_from_other.size());

    // Draw keypoints as circles.
    drawKeypoints(underground, kps_in_this, underground, Scalar(255, 0, 0),
                  DrawMatchesFlags::DRAW_OVER_OUTIMG);
    drawKeypoints(underground, kps_from_other, underground, Scalar(0, 0, 255),
                  DrawMatchesFlags::DRAW_OVER_OUTIMG);

    // Draw lines between matching keypoints to visualize their distance in
    // pixel.
    for (std::size_t i = 0UL; i < kps_in_this.size(); ++i) {
        if (invalid_keypoint(kps_from_other[i].pt, feature_file.w(),
                             feature_file.h()))
            continue;

        line(underground, kps_in_this[i].pt, kps_from_other[i].pt,
             /*color=*/Scalar(0, 255, 0),
             /*thickness=*/2,
             /*linetype=*/LINE_AA);
    }
    return underground;
}
}  // namespace sens_loc::plot
