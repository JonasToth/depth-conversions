#include "precision_recall.h"

#include "icp.h"
#include "keypoint_transform.h"

#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <iterator>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
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
#include <sens_loc/plot/backprojection.h>
#include <sens_loc/util/console.h>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

using namespace std;
using namespace sens_loc;
using namespace gsl;
using namespace cv;

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

template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
class prec_recall_analysis {
  public:
    prec_recall_analysis(string_view              feature_file_pattern,
                         string_view              depth_image_pattern,
                         string_view              pose_file_pattern,
                         float                    unit_factor,
                         string_view              intrinsic_file,
                         NormTypes                matching_norm,
                         not_null<mutex*>         distance_mutex,
                         not_null<vector<float>*> global_distances,
                         optional<string_view>    backproject_pattern,
                         optional<string_view>    original_files) noexcept
        : _feature_file_pattern{feature_file_pattern}
        , _depth_image_pattern{depth_image_pattern}
        , _pose_file_pattern{pose_file_pattern}
        , _unit_factor{unit_factor}
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
            _icp = rgbd::FastICPOdometry::create(
                apps::cv_camera_matrix(_intrinsic));
        }
    }

    void operator()(int idx,
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    optional<vector<KeyPoint>> keypoints,
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    optional<Mat> descriptors) noexcept try {
        Expects(!keypoints);
        Expects(!descriptors);

        const int previous_idx = idx - 1;

        using namespace math;
        using namespace apps;

        const reprojection_data prev{
            fmt::format(_feature_file_pattern, previous_idx),
            fmt::format(_depth_image_pattern, previous_idx),
            fmt::format(_pose_file_pattern, previous_idx)};
        const reprojection_data curr{fmt::format(_feature_file_pattern, idx),
                                     fmt::format(_depth_image_pattern, idx),
                                     fmt::format(_pose_file_pattern, idx)};

        // == Calculate relative pose between the two frames.
        pose_t rel_pose = relative_pose(prev.absolute_pose, curr.absolute_pose);

        // Refine that pose with an ICP if possible.
        if (_icp) {
            auto [icp_pose, icp_success] =
                refine_pose(*_icp, prev.depth_image, curr.depth_image,
                            _unit_factor, rel_pose);
            if (icp_success) {
                rel_pose = icp_pose;
            } else {
                lock_guard g{this->_stdio_mutex};
                cerr << util::warn{} << "No ICP result for idx: " << idx
                     << "!\n";
            }
        }

        // == Match the keypoints with cross-checking.
        vector<DMatch> matches;
        // QueryDescriptors: first argument
        // TrainDescriptors: second argument
        _matcher->match(curr.descriptors, prev.descriptors, matches);

        // == get keypoints as world points
        // Extract the pixels that are the center of the keypoint and use
        // their depth value to create the 3D point.
        // Trainpointcloud: Previous
        const auto [kps, prev_kps] =
            analysis::gather_matches(matches, curr.keypoints, prev.keypoints);
        Expects(kps.size() == prev_kps.size());

        pointcloud_t prev_points = keypoints_to_pointcloud(
            prev_kps, prev.depth_image, _intrinsic, _unit_factor);

        imagepoints_t prev_in_img =
            project_to_other_camera(rel_pose, prev_points, _intrinsic);
        Expects(prev_in_img.size() == prev_kps.size());

        // == Calculate the distance between the previous keypoints
        // backprojected in the current image and the keypoints in this image.
        imagepoints_t img_kps   = camera_models::keypoint_to_coords(kps);
        auto          distances = pointwise_distance(img_kps, prev_in_img);

        if (_backproject_pattern) {
            optional<math::image<uchar>> orig_img =
                load_file(fmt::format(*_original_files, idx));
            if (orig_img) {
                Mat out_img = plot::backprojection_correspondence(
                    *orig_img, img_kps, prev_in_img);
                imwrite(fmt::format(*_backproject_pattern, idx), out_img);
            } else {
                lock_guard g{this->_stdio_mutex};
                cerr << util::warn{} << "Original file for: " << idx
                     << "could not be loaded for backprojection plot!\n";
            }
        }

        {
            lock_guard guard{*_distance_mutex};
            _global_distances->insert(_global_distances->end(),
                                      distances.begin(), distances.end());
        }
    } catch (const std::exception& e) {
        lock_guard g{_stdio_mutex};
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
    float          _unit_factor;
    Model<Real>    _intrinsic;
    Ptr<BFMatcher> _matcher;

    static mutex             _stdio_mutex;
    not_null<mutex*>         _distance_mutex;
    not_null<vector<float>*> _global_distances;

    optional<string_view> _backproject_pattern;
    optional<string_view> _original_files;

    Ptr<rgbd::Odometry> _icp;
};
template <template <typename> typename Model, typename Real>
mutex prec_recall_analysis<Model, Real>::_stdio_mutex{};

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
    const float   unit_factor = 0.001F;
    auto          analysis_v  = visitor{feature_file_pattern,
                              feature_file_pattern,
                              depth_image_pattern,
                              pose_file_pattern,
                              unit_factor,
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
