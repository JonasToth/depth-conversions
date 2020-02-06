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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/analysis/match.h>
#include <sens_loc/analysis/precision_recall.h>
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
#include <util/statistic_visitor.h>

using namespace std;
using namespace sens_loc;
using namespace gsl;
using namespace cv;

namespace {

/// Capsulate all required data for back-and-forth projection as well
/// as precision-recall computation.
struct reprojection_data {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    math::image<ushort>       depth_image;
    math::pose_t              absolute_pose;

    reprojection_data(std::string_view feature_path,
                      std::string_view depth_path,
                      std::string_view pose_path) noexcept(false) {
        const FileStorage fs = io::open_feature_file(string(feature_path));
        keypoints            = io::load_keypoints(fs);
        descriptors          = io::load_descriptors(fs);

        optional<math::image<ushort>> d_img =
            io::load_image<ushort>(string(depth_path), IMREAD_UNCHANGED);
        if (!d_img) {
            ostringstream oss;
            oss << "Could not load depth image from " << depth_path << "!";
            throw std::runtime_error{oss.str()};
        }
        depth_image = move(*d_img);

        ifstream               pose_file{string(pose_path)};
        optional<math::pose_t> pose = io::load_pose(pose_file);
        if (!pose) {
            ostringstream oss;
            oss << "Could not load pose from " << pose_path << "!";
            throw std::runtime_error{oss.str()};
        }
        absolute_pose = move(*pose);
    }
};

size_t mask_backprojection(const math::image<uchar>& mask,
                           math::imagepoints_t&      points) noexcept {
    size_t counter = 0UL;
    for (math::pixel_coord<float>& p : points) {
        if (p.u() < 0.0F || p.v() < 0.0F ||
            p.u() > narrow_cast<float>(mask.w()) ||
            p.v() > narrow_cast<float>(mask.h()) || mask.at(p) == 0U) {
            p = {-1.0F, -1.0F};
            ++counter;
        }
    }
    return counter;
}

template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
class prec_recall_analysis {
  public:
    prec_recall_analysis(
        string_view              feature_file_pattern,
        string_view              depth_image_pattern,
        string_view              pose_file_pattern,
        float                    unit_factor,
        string_view              intrinsic_file,
        optional<string_view>    mask_file,
        NormTypes                matching_norm,
        float                    keypoint_distances_threshold,
        not_null<mutex*>         inserter_mutex,
        not_null<vector<float>*> selected_elements_distance,
        not_null<analysis::precision_recall_statistic*> prec_recall_stat,
        not_null<size_t*>                               totally_masked,
        optional<string_view>                           backproject_pattern,
        optional<string_view>                           original_files) noexcept
        : _feature_file_pattern{feature_file_pattern}
        , _depth_image_pattern{depth_image_pattern}
        , _pose_file_pattern{pose_file_pattern}
        , _unit_factor{unit_factor}
        , _keypoint_distance_threshold{keypoint_distances_threshold}
        , _matcher{cv::BFMatcher::create(matching_norm, /*crosscheck=*/true)}
        , _inserter_mutex{inserter_mutex}
        , _selected_elements_dist{selected_elements_distance}
        , _stats{prec_recall_stat}
        , _totally_masked{totally_masked}
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

        if (mask_file) {
            _mask = io::load_image<uchar>(string(*mask_file), IMREAD_GRAYSCALE);
            if (!_mask) {
                cerr << util::err{} << "Could not load mask file '"
                     << *mask_file << "'!\n"
                     << "Continuing without masking.\n";
            }
            // The mask file is loaded. It is necessary to check, that the
            // dimensions match the loaded intrinsic. If not, masking is
            // disabled.
            else {
                if (_mask->w() != _intrinsic.w() ||
                    _mask->h() != _intrinsic.h()) {
                    cerr << util::err{}
                         << "Dimensions of the mask mismatch the dimensions of "
                            "the intrinsic!\n"
                         << "Disabling masking.\n"
                         << "Mask {" << _mask->w() << ", " << _mask->h()
                         << "} vs Intrinsic {" << _intrinsic.w() << ", "
                         << _intrinsic.h() << "}\n";
                    _mask = nullopt;
                }
            }
        } else
            _mask = nullopt;

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

        reprojection_data prev{fmt::format(_feature_file_pattern, previous_idx),
                               fmt::format(_depth_image_pattern, previous_idx),
                               fmt::format(_pose_file_pattern, previous_idx)};
        reprojection_data curr{fmt::format(_feature_file_pattern, idx),
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

        // == get keypoints as world points
        pointcloud_t prev_points = keypoints_to_pointcloud(
            prev.keypoints, prev.depth_image, _intrinsic, _unit_factor);

        imagepoints_t prev_in_img =
            project_to_other_camera(rel_pose, prev_points, _intrinsic);

        // Some backprojected points might land outside of the viewport of
        // the camera model, but still on valid pixel coordinates (inside the
        // image). This can be detected with using a mask.
        // Such points are set to {-1, -1} to be recognizable invalid.
        const size_t masked_points =
            _mask ? mask_backprojection(*_mask, prev_in_img) : 0;
        Expects(prev_in_img.size() == prev.keypoints.size());

        // == Match the keypoints with cross-checking.
        vector<DMatch> matches;
        // QueryDescriptors: first argument
        // TrainDescriptors: second argument
        _matcher->match(curr.descriptors, prev.descriptors, matches);

        using analysis::element_categories;
        using camera_models::keypoint_to_coords;
        const imagepoints_t curr_keypoints = keypoint_to_coords(curr.keypoints);

        const element_categories classification(
            curr_keypoints, prev_in_img, matches, _keypoint_distance_threshold);
        // True positives keypoints from this and previous frame in this
        // frames coordinate system.
        auto [t_p_t, t_p_o] = analysis::gather_correspondences(
            classification.true_positives, curr_keypoints, prev_in_img);

        if (_backproject_pattern) {
            optional<math::image<uchar>> orig_img =
                io::load_as_8bit_gray(fmt::format(*_original_files, idx));

            if (orig_img) {
                // Plot true positives.
                Mat out_img = plot::backprojection_correspondence(
                    orig_img->data(), t_p_t, t_p_o,  // Keypoints
                    CV_RGB(0, 120, 0),               // Dark Green
                    6                                // Thickness
                );

                // Plot false negatives with purple line
                auto [f_n_t, f_n_o] = analysis::gather_correspondences(
                    classification.false_negatives, curr_keypoints,
                    prev_in_img);
                out_img = plot::backprojection_correspondence(
                    out_img, f_n_t, f_n_o,  // Base image and keypoints
                    CV_RGB(30, 39, 140),    // Dark Blueish
                    6                       // Thickness
                );

                // Plot false positives with orange distance line
                auto [f_p_t, f_p_o] = analysis::gather_correspondences(
                    classification.false_positives, curr_keypoints,
                    prev_in_img);
                out_img = plot::backprojection_correspondence(
                    out_img, f_p_t, f_p_o,  // Base image and keypoints
                    CV_RGB(245, 130, 50),   // ocre orange
                    1                       // Thickness
                );


                const bool write_success =
                    imwrite(fmt::format(*_backproject_pattern, idx), out_img);

                if (!write_success) {
                    lock_guard g{this->_stdio_mutex};
                    cerr << util::err{}
                         << "Could not write backprojection correspondence to "
                            "disk for: "
                         << idx << "!\n";
                }
            } else {
                lock_guard g{this->_stdio_mutex};
                cerr << util::warn{} << "Original file for: " << idx
                     << "could not be loaded for backprojection plot!\n";
            }
        }

        auto distances = pointwise_distance(t_p_t, t_p_o);

        {
            lock_guard guard{*_inserter_mutex};
            _selected_elements_dist->insert(_selected_elements_dist->end(),
                                            distances.begin(), distances.end());
            _stats->account(classification);
            *_totally_masked += masked_points;
        }
    } catch (const std::exception& e) {
        lock_guard g{_stdio_mutex};
        cerr << util::err{} << "Could not analyze data for " << idx << "!\n"
             << e.what() << "\n";
        return;
    }

    void postprocess() noexcept {
        lock_guard guard{*_inserter_mutex};

        const auto         dist_bins = 20;
        analysis::distance distance_stat{*_selected_elements_dist, dist_bins,
                                         "backprojection error pixels"};

        cout << distance_stat.histogram() << "\n"
             << "# relevant:   " << distance_stat.count() << "\n"
             << "min:          " << distance_stat.min() << "\n"
             << "max:          " << distance_stat.max() << "\n"
             << "median:       " << distance_stat.median() << "\n"
             << "mean:         " << distance_stat.mean() << "\n"
             << "Variance:     " << distance_stat.variance() << "\n"
             << "StdDev:       " << distance_stat.stddev() << "\n"
             << "Skewness:     " << distance_stat.skewness() << "\n"
             << "Precision     " << _stats->precision() << "\n"
             << "Recall:       " << _stats->recall() << "\n"
             << "Specificity:  " << _stats->specificity() << "\n"
             << "Rand-Index:   " << _stats->rand_index() << "\n"
             << "Youden-Index: " << _stats->youden_index() << "\n"
             << "Masked pts:   " << *_totally_masked << "\n";
    }

  private:
    string_view                  _feature_file_pattern;
    string_view                  _depth_image_pattern;
    string_view                  _pose_file_pattern;
    float                        _unit_factor;
    float                        _keypoint_distance_threshold;
    Model<Real>                  _intrinsic;
    Ptr<BFMatcher>               _matcher;
    optional<math::image<uchar>> _mask;

    static mutex _stdio_mutex;

    not_null<mutex*>                                _inserter_mutex;
    not_null<vector<float>*>                        _selected_elements_dist;
    not_null<analysis::precision_recall_statistic*> _stats;
    not_null<size_t*>                               _totally_masked;

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
                             optional<string_view> mask_file,
                             NormTypes             matching_norm,
                             float                 keypoint_distances_threshold,
                             optional<string_view> backproject_pattern,
                             optional<string_view> original_files) {
    Expects(start_idx < end_idx &&
            "Precision-Recall calculation requires at least two images");

    // The code will load all data directly and does not rely on loading through
    // the statistics code.
    using visitor =
        statistic_visitor<prec_recall_analysis<>, required_data::none>;

    mutex                                inserter_mutex;
    vector<float>                        selected_elements_distance;
    analysis::precision_recall_statistic stats;
    size_t                               totally_masked = 0UL;

    const float unit_factor = 0.001F;
    auto        analysis_v  = visitor{feature_file_pattern,
                              feature_file_pattern,
                              depth_image_pattern,
                              pose_file_pattern,
                              unit_factor,
                              intrinsic_file,
                              mask_file,
                              matching_norm,
                              keypoint_distances_threshold,
                              not_null{&inserter_mutex},
                              not_null{&selected_elements_distance},
                              not_null{&stats},
                              not_null{&totally_masked},
                              backproject_pattern,
                              original_files};

    // Consecutive images are matched and analysed, therefore the first
    // index must be skipped.
    auto f = parallel_visitation(start_idx + 1, end_idx, analysis_v);
    f.postprocess();

    return 0;
}

}  // namespace sens_loc::apps
