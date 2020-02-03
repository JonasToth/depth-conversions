#include "precision_recall.h"

#include "icp.h"

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

#define DEBUG_OUT 0

namespace {

struct reprojection_data {
    vector<KeyPoint>    keypoints;
    Mat                 descriptors;
    math::image<ushort> depth_image;
    math::pose_t        absolute_pose;

    reprojection_data(string_view feature_path,
                      string_view depth_path,
                      string_view pose_path) {
        const FileStorage fs = apps::open_feature_file(string(feature_path));
        keypoints            = apps::load_keypoints(fs);
        descriptors          = apps::load_descriptors(fs);

        optional<math::image<ushort>> d_img =
            io::load_image<ushort>(string(depth_path), cv::IMREAD_UNCHANGED);
        if (!d_img)
            throw std::runtime_error{
                fmt::format("Could not load depth image from {}!", depth_path)};
        depth_image = move(*d_img);

        ifstream               pose_file{string(pose_path)};
        optional<math::pose_t> pose = io::load_pose(pose_file);
        if (!pose)
            throw std::runtime_error{
                fmt::format("Could not load pose from {}!", pose_path)};
        absolute_pose = move(*pose);
    }
};

#if DEBUG_OUT
inline void plot_keypoints(string_view             background_file,
                           const vector<KeyPoint>& kps,
                           const Scalar&           color,
                           string_view             output_file) {
    auto orig_img = apps::load_file(string(background_file));
    Mat  out_img;
    cvtColor(orig_img->data(), out_img, cv::COLOR_GRAY2BGR);
    // Draw the keypoints detected in this image.
    drawKeypoints(out_img, kps, out_img, color,
                  DrawMatchesFlags::DRAW_OVER_OUTIMG);
    imwrite(string(output_file), out_img);
}
#endif

template <typename Real = float>
inline Mat cv_camera_matrix(const camera_models::pinhole<Real>& i) {
    Mat K             = Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = i.fx();
    K.at<float>(1, 1) = i.fy();
    K.at<float>(0, 2) = i.cx();
    K.at<float>(1, 2) = i.cy();
    return K;
}
constexpr float unit_factor = 0.001F;
mutex           stdio_mutex;

template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
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
                                camera_models::pinhole<Real>>) {
            _icp = rgbd::FastICPOdometry::create(cv_camera_matrix(_intrinsic));
        }
    }

    void operator()(int idx,
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    optional<vector<KeyPoint>> keypoints,
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    optional<Mat> descriptors) noexcept try {
        Expects(!keypoints);
        Expects(!descriptors);
#if DEBUG_OUT
        lock_guard guard{stdio_mutex};
#endif

        const int previous_idx = idx - 1;

        using namespace camera_models;
        using namespace math;
        using namespace apps;

        reprojection_data prev{fmt::format(_feature_file_pattern, previous_idx),
                               fmt::format(_depth_image_pattern, previous_idx),
                               fmt::format(_pose_file_pattern, previous_idx)};
        reprojection_data curr{fmt::format(_feature_file_pattern, idx),
                               fmt::format(_depth_image_pattern, idx),
                               fmt::format(_pose_file_pattern, idx)};

        // == Match the keypoints with cross-checking.
        vector<DMatch> matches;
        // QueryDescriptors: first argument
        // TrainDescriptors: second argument
        _matcher->match(curr.descriptors, prev.descriptors, matches);
#if DEBUG_OUT
        cerr << "First pixel after match from idx " << idx << ":\n"
             << "Train: " << prev.keypoints[matches[0].trainIdx].pt << "\n"
             << "Query: " << curr.keypoints[matches[0].queryIdx].pt << "\n";
#endif

        // == get keypoints as world points
        // Extract the pixels that are the center of the keypoint and use
        // their depth value to create the 3D point.
        // Trainpointcloud: Previous
        const auto [kps, prev_kps] =
            analysis::gather_matches(matches, curr.keypoints, prev.keypoints);
#if DEBUG_OUT
        cerr << "After gathering\n"
             << "Train: " << prev_kps[0].pt << "\n"
             << "Query: " << kps[0].pt << "\n";

        // == Debug plot the keypoints from the gathered matches.
        plot_keypoints(fmt::format(*_original_files, idx), curr.keypoints,
                       Scalar(255, 0, 0),
                       fmt::format(*_backproject_pattern, idx));
        int idx_fun = 100 * previous_idx + idx;
        plot_keypoints(fmt::format(*_original_files, previous_idx),
                       prev.keypoints, Scalar(0, 0, 255),
                       fmt::format(*_backproject_pattern, idx_fun));
        return;
#endif

        imagepoints_t img_previous = keypoint_to_coords(prev_kps);
#if DEBUG_OUT
        cerr << "pixel coordinates on previous image: " << img_previous[0].u()
             << " / " << img_previous[0].v() << "\n";
#endif

        pointcloud_t prev_points = project_to_sphere(_intrinsic, img_previous);
#if DEBUG_OUT
        cerr << "Project to sphere: " << prev_points[0].X() << " / "
             << prev_points[0].Y() << " / " << prev_points[0].Z() << "\n";
#endif

        Expects(kps.size() == prev_kps.size());
        for (unsigned int match_idx = 0; match_idx < kps.size(); ++match_idx) {
            auto orig_depth = prev.depth_image.at(img_previous[match_idx]);
            auto t_d        = unit_factor * narrow_cast<float>(orig_depth);
            prev_points[match_idx] = t_d * prev_points[match_idx];
        }

#if DEBUG_OUT
        cerr << "Project to Space: " << prev_points[0].X() << " / "
             << prev_points[0].Y() << " / " << prev_points[0].Z() << "\n";

        imagepoints_t prev_backprojected_to_prev =
            project_to_image(_intrinsic, prev_points);
        cerr << "pixel coordinates on previous image backprojected: "
             << prev_backprojected_to_prev[0].u() << " / "
             << prev_backprojected_to_prev[0].v() << "\n";

        vector<KeyPoint> prev_back_img =
            coords_to_keypoint(prev_backprojected_to_prev);
        int idx_fun = 100 * previous_idx + idx;
        plot_keypoints(fmt::format(*_original_files, previous_idx),
                       prev_back_img, Scalar(70, 70, 180),
                       fmt::format(*_backproject_pattern, idx_fun));
#endif

        // == Calculate relative pose between the two frames.
        pose_t rel_pose = relative_pose(prev.absolute_pose, curr.absolute_pose);

        // Refine that pose with an ICP if possible.
        if (_icp) {
            auto [icp_pose, icp_success] =
                refine_pose(*_icp, prev.depth_image, curr.depth_image,
                            unit_factor, rel_pose);
            if (icp_success) {
                rel_pose = icp_pose;
            } else {
                lock_guard g{stdio_mutex};
                cerr << util::warn{} << "No ICP result for idx: " << idx
                     << "!\n";
            }
        }

        // == Transform matches from previous frame into this coordinate
        // frame.
        pointcloud_t prev_in_this_frame = rel_pose * prev_points;
#if DEBUG_OUT
        cerr << "Project into this frame: " << prev_in_this_frame[0].X()
             << " / " << prev_in_this_frame[0].Y() << " / "
             << prev_in_this_frame[0].Z() << "\n";
#endif

        // == Project the points back into the current image.
        imagepoints_t prev_in_img =
            project_to_image(_intrinsic, prev_in_this_frame);
#if DEBUG_OUT
        cerr << "Pixel in this frame: " << prev_in_img[0].u() << " / "
             << prev_in_img[0].v() << "\n";
#endif

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

            Ensures(kps.size() == prev_in_this_kps.size());
            for (std::size_t i = 0UL; i < kps.size(); ++i) {
                line(out_img, kps[i].pt, prev_in_this_kps[i].pt,
                     /*color=*/Scalar(0, 255, 0),
                     /*thickness=*/2,
                     /*linetype=*/LINE_AA);
            }

            imwrite(fmt::format(*_backproject_pattern, idx), out_img);
        }

        {
            lock_guard guard{*_distance_mutex};
            _global_distances->insert(_global_distances->end(),
                                      distances.begin(), distances.end());
        }
    } catch (const std::exception& e) {
        lock_guard g{stdio_mutex};
        cerr << util::err{} << "Could not analyze data for " << idx << "!\n"
             << e.what() << "\n";
        return;
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
    string_view    _feature_file_pattern;
    string_view    _depth_image_pattern;
    string_view    _pose_file_pattern;
    Model<Real>    _intrinsic;
    Ptr<BFMatcher> _matcher;

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

    // The code will load all data directly and does not rely on loading through
    // the statistics code.
    using visitor =
        statistic_visitor<prec_recall_analysis<>, required_data::none>;
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
