#include "precision_recall.h"

#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/analysis/match.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/io/pose.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/image.h>
#include <sens_loc/math/pointcloud.h>
#include <sens_loc/util/console.h>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

using namespace std;
using namespace sens_loc;
using namespace gsl;
using namespace cv;

namespace {

inline Mat cv_camera_matrix(const camera_models::pinhole<float>& i) {
    Mat K             = Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = i.fx();
    K.at<float>(1, 1) = i.fy();
    K.at<float>(0, 2) = i.cx();
    K.at<float>(1, 2) = i.cy();
    return K;
}

constexpr float unit_factor = 0.001F;

class prec_recall_analysis {
  public:
    prec_recall_analysis(string_view              feature_file_pattern,
                         string_view              depth_image_pattern,
                         string_view              pose_file_pattern,
                         string_view              intrinsic_file,
                         NormTypes                matching_norm,
                         not_null<mutex*>         distance_mutex,
                         not_null<vector<float>*> global_distances,
                         optional<string_view>    backproject_pattern,
                         optional<string_view>    original_files) noexcept
        : _feature_file_pattern{feature_file_pattern}
        , _depth_image_pattern{depth_image_pattern}
        , _pose_file_pattern{pose_file_pattern}
        , _matcher{cv::BFMatcher::create(matching_norm, /*crosscheck=*/true)}
        , _distance_mutex{distance_mutex}
        , _global_distances{global_distances}
        , _backproject_pattern{backproject_pattern}
        , _original_files{original_files} {
        Expects(!_feature_file_pattern.empty());
        Expects(!_depth_image_pattern.empty());
        Expects(!_pose_file_pattern.empty());
        Expects(!intrinsic_file.empty());

        ifstream intrinsic{string(intrinsic_file)};

        auto maybe_intrinsic =
            io::camera<float, camera_models::pinhole>::load_intrinsic(
                intrinsic);
        Expects(maybe_intrinsic.has_value());
        _intrinsic = *maybe_intrinsic;

        if constexpr (is_same_v<decltype(_intrinsic),
                                camera_models::pinhole<float>>) {
            _icp = rgbd::ICPOdometry::create(cv_camera_matrix(_intrinsic)
#if 0
                ,
                /*minDepth=*/0.1F,      // Distance in meters.
                /*maxDepth=*/10.0F,     // Distance in meters.
                /*maxDepthDiff=*/0.05F  // Distance in meters.
#endif
            );
        }
    }

    void operator()(int                        idx,
                    optional<vector<KeyPoint>> keypoints,
                    optional<Mat>              descriptors) noexcept {
        Expects(keypoints);
        Expects(!keypoints->empty());
        Expects(descriptors);
        Expects(size_t(descriptors->rows) == keypoints->size());

        static mutex stdio_mutex;
        const int    previous_idx = idx - 1;

        using namespace camera_models;
        using namespace math;
        using namespace apps;

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

        lock_guard guard{stdio_mutex};

        // == Match the keypoints with cross-checking.
        vector<DMatch> matches;
        // QueryDescriptors: first argument
        // TrainDescriptors: second argument
        _matcher->match(previous_descriptors, *descriptors, matches);
        cerr << "First pixel after match:\n"
             << "Train: " << previous_keypoints[matches[0].trainIdx].pt << "\n"
             << "Query: " << (*keypoints)[matches[0].queryIdx].pt << "\n";

        // == get keypoints as world points
        // Extract the pixels that are the center of the keypoint and use
        // their depth value to create the 3D point.
        // Trainpointcloud: Previous
        const auto [kps, prev_kps] =
            analysis::gather_matches(matches, previous_keypoints, *keypoints);

        imagepoints_t img_previous = keypoint_to_coords(prev_kps);
        cerr << "Track first point in " << idx
             << ":\npixel coordinates on previous image: "
             << img_previous[0].u() << " / " << img_previous[0].v() << "\n";

        pointcloud_t prev_points = project_to_sphere(_intrinsic, img_previous);
        cerr << "Project to sphere: " << prev_points[0].X() << " / "
             << prev_points[0].Y() << " / " << prev_points[0].Z() << "\n";

        Expects(kps.size() == prev_kps.size());
        for (unsigned int match_idx = 0; match_idx < kps.size(); ++match_idx) {
            // Retrieve the depth value from the input depth-image for the
            // previous index.
            auto t_kp = prev_kps[match_idx];
            auto t_px = math::pixel_coord<float>{t_kp.pt.x, t_kp.pt.y};
            auto t_d  = narrow_cast<float>(previous_depth_image->at(t_px));
            prev_points[match_idx] = t_d * prev_points[match_idx];
        }
        cerr << "Project to Space: " << prev_points[0].X() << " / "
             << prev_points[0].Y() << " / " << prev_points[0].Z() << "\n";

        // == Calculate relative pose between the two frames.
        pose_t rel_pose = relative_pose(*previous_pose, *pose);
        for (int i = 0; i < 3; ++i)
            rel_pose(i, 3) *= 1.0F / unit_factor;

#if 0
        // Refine that pose with an ICP if possible.
        if (_icp) {

            Mat Rt;
            Mat initial = Mat::eye(4, 4, CV_64FC1);
            // Copy rotation part.
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    initial.at<double>(i, j) = rel_pose(i, j);

            Mat prev_mask;
            previous_depth_image->data().convertTo(prev_mask, CV_8UC1);

            Mat this_mask;
            depth_image->data().convertTo(this_mask, CV_8UC1);

            Mat cvt_prev_depth;
            previous_depth_image->data().convertTo(cvt_prev_depth, CV_32F,
                                                   unit_factor);

            Mat cvt_this_depth;
            depth_image->data().convertTo(cvt_this_depth, CV_32F, unit_factor);

            const bool icp_success = _icp->compute(/*srcImage=*/Mat(),
                                                   /*srcDepth=*/cvt_prev_depth,
                                                   /*srcMask=*/prev_mask,
                                                   /*dstImage=*/Mat(),
                                                   /*dstDepth=*/cvt_this_depth,
                                                   /*dstMask=*/this_mask,
                                                   /*Rt=*/Rt,
                                                   /*initRt=*/initial);

            if (icp_success) {
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        rel_pose(i, j) = Rt.at<double>(i, j);
            } else {
                cerr << util::warn{} << "No ICP result for idx: " << idx
                     << "!\n";
            }
        }
#endif

        cerr << rel_pose << "\n";
        // == Transform matches from previous frame into this coordinate
        // frame.
        pointcloud_t prev_in_this_frame = rel_pose * prev_points;
        cerr << "Project into this frame: " << prev_in_this_frame[0].X()
             << " / " << prev_in_this_frame[0].Y() << " / "
             << prev_in_this_frame[0].Z() << "\n";

        // == Project the points back into the current image.
        imagepoints_t prev_in_img =
            project_to_image(_intrinsic, prev_in_this_frame);
        cerr << "Pixel in this frame: " << prev_in_img[0].u() << " / "
             << prev_in_img[0].v() << "\n";

        // == Calculate the distance between the previous keypoints
        // backprojected in the current image and the keypoints in this image.
        imagepoints_t img_kps   = keypoint_to_coords(kps);
        auto          distances = pointwise_distance(img_kps, prev_in_img);

        if (_backproject_pattern) {
            // == Plot the keypoints in one image.
            auto orig_img = load_file(fmt::format(*_original_files, idx));
            vector<KeyPoint> prev_in_this_kps = coords_to_keypoint(prev_in_img);

            Mat out_img;
            cvtColor(orig_img->data(), out_img, cv::COLOR_GRAY2BGR);
            // Draw the keypoints detected in this image.
            drawKeypoints(out_img, kps, out_img, Scalar(255, 0, 0),
                          DrawMatchesFlags::DRAW_OVER_OUTIMG);
            // Draw the keypoints detected in the previous image,
            // backprojected into this image.
            drawKeypoints(out_img, prev_in_this_kps, out_img, Scalar(0, 0, 255),
                          DrawMatchesFlags::DRAW_OVER_OUTIMG);
            imwrite(fmt::format(*_backproject_pattern, idx), out_img);
        }

        {
            lock_guard guard{*_distance_mutex};
            _global_distances->insert(_global_distances->end(),
                                      distances.begin(), distances.end());
        }
    }

    void postprocess() noexcept {
        lock_guard guard{*_distance_mutex};

        // Calculate the median of the distances by partial sorting until
        // the middle element.
        auto median_it =
            _global_distances->begin() + _global_distances->size() / 2UL;
        nth_element(_global_distances->begin(), median_it,
                    _global_distances->end());
        _global_distances->erase(median_it, _global_distances->end());

        const auto         dist_bins = 50;
        analysis::distance distance_stat{*_global_distances, dist_bins,
                                         "backprojection error pixels"};

#if 0
        cout << distance_stat.histogram() << "\n"
             << "matched count:  " << distance_stat.count() << "\n"
             << "min:            " << distance_stat.min() << "\n"
             << "max:            " << distance_stat.max() << "\n"
             << "median:         " << distance_stat.median() << "\n"
             << "mean:           " << distance_stat.mean() << "\n"
             << "Variance:       " << distance_stat.variance() << "\n"
             << "StdDev:         " << distance_stat.stddev() << "\n"
             << "Skewness:       " << distance_stat.skewness() << "\n";
#endif
    }

  private:
    string_view                   _feature_file_pattern;
    string_view                   _depth_image_pattern;
    string_view                   _pose_file_pattern;
    camera_models::pinhole<float> _intrinsic;
    Ptr<BFMatcher>                _matcher;

    not_null<mutex*>         _distance_mutex;
    not_null<vector<float>*> _global_distances;

    optional<string_view> _backproject_pattern;
    optional<string_view> _original_files;

    Ptr<rgbd::Odometry> _icp;
};
}  // namespace

namespace sens_loc::apps {
int analyze_precision_recall(string_view           feature_file_pattern,
                             int                   start_idx,
                             int                   end_idx,
                             string_view           depth_image_pattern,
                             string_view           pose_file_pattern,
                             string_view           intrinsic_file,
                             NormTypes             matching_norm,
                             optional<string_view> backproject_pattern,
                             optional<string_view> original_files) {
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
                              not_null{&global_distances},
                              backproject_pattern,
                              original_files};

    // Consecutive images are matched and analysed, therefore the first
    // index must be skipped.
    auto f = parallel_visitation(start_idx + 1, end_idx, analysis_v);
    f.postprocess();

    return 0;
}

}  // namespace sens_loc::apps
