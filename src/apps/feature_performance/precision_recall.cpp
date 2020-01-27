#include "precision_recall.h"

#include "sens_loc/math/image.h"

#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/analysis/match.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/io/pose.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/pointcloud.h>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

using namespace std;
using namespace gsl;

namespace {

class prec_recall_analysis {
  public:
    prec_recall_analysis(string_view              feature_file_pattern,
                         string_view              depth_image_pattern,
                         string_view              pose_file_pattern,
                         string_view              intrinsic_file,
                         cv::NormTypes            matching_norm,
                         not_null<mutex*>         distance_mutex,
                         not_null<vector<float>*> global_distances) noexcept
        : _feature_file_pattern{feature_file_pattern}
        , _depth_image_pattern{depth_image_pattern}
        , _pose_file_pattern{pose_file_pattern}
        , _matcher{cv::BFMatcher::create(matching_norm, /*crosscheck=*/true)}
        , _distance_mutex{distance_mutex}
        , _global_distances{global_distances} {
        Expects(!_feature_file_pattern.empty());
        Expects(!_depth_image_pattern.empty());
        Expects(!_pose_file_pattern.empty());
        Expects(!intrinsic_file.empty());

        ifstream intrinsic{string(intrinsic_file)};

        auto maybe_intrinsic = sens_loc::io::camera<
            float, sens_loc::camera_models::pinhole>::load_intrinsic(intrinsic);
        Expects(maybe_intrinsic.has_value());
        _intrinsic = *maybe_intrinsic;
    }

    void operator()(int                            idx,
                    optional<vector<cv::KeyPoint>> keypoints,
                    optional<cv::Mat>              descriptors) noexcept {
        Expects(keypoints);
        Expects(!keypoints->empty());
        Expects(descriptors);
        Expects(size_t(descriptors->rows) == keypoints->size());

        const int previous_idx = idx - 1;

        using namespace sens_loc;
        using namespace camera_models;
        using namespace math;
        using namespace sens_loc::apps;
        using namespace std;
        using namespace cv;
        using namespace Eigen;

        // == Load all data from the previous index.
        const FileStorage previous_feature_file =
            open_feature_file(_feature_file_pattern, previous_idx);
        vector<KeyPoint> previous_keypoints =
            load_keypoints(previous_feature_file);
        Mat previous_descriptors = load_descriptors(previous_feature_file);

        const string previous_depth_path =
            fmt::format(_depth_image_pattern, previous_idx);
        optional<math::image<ushort>> previous_depth_image =
            io::load_image<ushort>(previous_depth_path, cv::IMREAD_UNCHANGED);
        if (!previous_depth_image) {
            cerr << "No previous depth image\n";
            return;
        }

        const string previous_pose_path =
            fmt::format(_pose_file_pattern, previous_idx);
        ifstream         previous_pose_file(previous_pose_path);
        optional<pose_t> previous_pose = io::load_pose(previous_pose_file);
        if (!previous_pose) {
            cerr << "No previous pose - " << previous_pose_path << "\n";
            return;
        }

        // == Load the missing data for the current index.
        const string depth_path = fmt::format(_depth_image_pattern, idx);
        optional<math::image<ushort>> depth_image =
            io::load_image<ushort>(depth_path, cv::IMREAD_UNCHANGED);
        if (!depth_image) {
            cerr << "No depth image\n";
            return;
        }

        const string     pose_path = fmt::format(_pose_file_pattern, idx);
        ifstream         pose_file(pose_path);
        optional<pose_t> pose = io::load_pose(pose_file);
        if (!pose) {
            cerr << "No pose - " << pose_path << "\n";
            return;
        }

        // == Match the keypoints with cross-checking.
        vector<DMatch> matches;
        // QueryDescriptors: first argument
        // TrainDescriptors: second argument
        _matcher->match(*descriptors, previous_descriptors, matches);

        // == get keypoints as world points
        // Extract the pixels that are the center of the keypoint and use
        // their depth value to create the 3D point.
        // Trainpointcloud: Previous
        const auto [kps, prev_kps] =
            analysis::gather_matches(matches, *keypoints, previous_keypoints);

        imagepoints_t img_previous = keypoint_to_coords(prev_kps);
        pointcloud_t  prev_points = project_to_sphere(_intrinsic, img_previous);

        Expects(kps.size() == prev_kps.size());
        for (unsigned int match_idx = 0; match_idx < kps.size(); ++match_idx) {
            auto t_kp = prev_kps[match_idx];
            auto t_px = math::pixel_coord<float>{t_kp.pt.x, t_kp.pt.y};
            auto t_d  = float(previous_depth_image->at(t_px));

            // Make coordinate homogeneous in camera the camera coordinates
            // of the previous camera.
            Vector4f cam_coords = t_d * prev_points.col(match_idx);
            cam_coords(3)       = 1.0F;

            prev_points.col(match_idx) = cam_coords;
        }

        // == Calculate relative pose between the two frames.
        pose_t rel_pose = relative_pose(*previous_pose, *pose);

        // == Transform matches from previous frame into this coordinate frame.
        pointcloud_t prev_in_this_frame = rel_pose * prev_points;

        // == Project the points back into the current image.
        imagepoints_t prev_in_img =
            project_to_image(_intrinsic, prev_in_this_frame);

        // == Calculate the distance between the keypoints backprojected
        imagepoints_t img_kps   = keypoint_to_coords(kps);
        auto          distances = pointwise_distance(img_kps, prev_in_img);

        // == Plot the keypoints in one image.
        vector<KeyPoint> prev_in_this_kps = coords_to_keypoint(prev_in_img);
        auto             cvt              = math::convert<uchar>(*depth_image);
        Mat              target;
        cvtColor(cvt.data(), target, COLOR_GRAY2BGR);
        // Draw the keypoints detected in this image.
        drawKeypoints(target, kps, target, Scalar(255, 0, 0),
                      DrawMatchesFlags::DRAW_OVER_OUTIMG);
        // Draw the keypoints detected in the previous image, backprojected
        // into this image.
        drawKeypoints(target, prev_in_this_kps, target, Scalar(0, 255, 0),
                      DrawMatchesFlags::DRAW_OVER_OUTIMG);
        imwrite(fmt::format("funk/coorespondence-{:04d}.png", idx), target);
#if 0
        // == Calculate the distance of each match in 3D space
        RowVectorXf distances = pointwise_distance(points, prev_in_this_frame);
        Ensures(size_t(distances.cols()) == matches.size());

        RowVectorXf::Index min_row;
        RowVectorXf::Index min_col;
        cout << "Min: " << distances.minCoeff(&min_row, &min_col)
                  << "  at pixel " << (*keypoints)[min_col].pt << "\n";
#endif

        {
            lock_guard guard{*_distance_mutex};
            for (int i = 0; i < distances.cols(); ++i)
                _global_distances->emplace_back(distances(i));
        }
    }

    void postprocess() noexcept {
        lock_guard guard{*_distance_mutex};

        // Calculate the median of the distances by partial sorting until the
        // middle element.
        auto median_it =
            _global_distances->begin() + _global_distances->size() / 2UL;
        nth_element(_global_distances->begin(), median_it,
                    _global_distances->end());
        _global_distances->erase(median_it, _global_distances->end());

        const auto                   dist_bins = 50;
        sens_loc::analysis::distance distance_stat{
            *_global_distances, dist_bins, "backprojection error pixels"};

        cout << distance_stat.histogram() << "\n"
             << "matched count:  " << distance_stat.count() << "\n"
             << "min:            " << distance_stat.min() << "\n"
             << "max:            " << distance_stat.max() << "\n"
             << "median:         " << distance_stat.median() << "\n"
             << "mean:           " << distance_stat.mean() << "\n"
             << "Variance:       " << distance_stat.variance() << "\n"
             << "StdDev:         " << distance_stat.stddev() << "\n"
             << "Skewness:       " << distance_stat.skewness() << "\n";
    }

  private:
    string_view                             _feature_file_pattern;
    string_view                             _depth_image_pattern;
    string_view                             _pose_file_pattern;
    sens_loc::camera_models::pinhole<float> _intrinsic;
    cv::Ptr<cv::BFMatcher>                  _matcher;

    not_null<mutex*>         _distance_mutex;
    not_null<vector<float>*> _global_distances;
};
}  // namespace

namespace sens_loc::apps {
int analyze_precision_recall(string_view   feature_file_pattern,
                             int           start_idx,
                             int           end_idx,
                             string_view   depth_image_pattern,
                             string_view   pose_file_pattern,
                             string_view   intrinsic_file,
                             cv::NormTypes matching_norm) {
    Expects(start_idx < end_idx &&
            "Precision-Recall calculation requires at least two images");

    using visitor =
        statistic_visitor<prec_recall_analysis, required_data::keypoints |
                                                    required_data::descriptors>;
    mutex         distance_mutex;
    vector<float> global_distances;

    auto analysis_v = visitor{feature_file_pattern,
                              feature_file_pattern,
                              depth_image_pattern,
                              pose_file_pattern,
                              intrinsic_file,
                              matching_norm,
                              not_null{&distance_mutex},
                              not_null{&global_distances}};

    // Consecutive images are matched and analysed, therefore the first index
    // must be skipped.
    auto f = parallel_visitation(start_idx + 1, end_idx, analysis_v);
    f.postprocess();

    return 0;
}

}  // namespace sens_loc::apps
