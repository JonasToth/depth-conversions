#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
#include "recognition_performance.h"

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
#include <sens_loc/analysis/recognition_performance.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/io/histogram.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/io/pose.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/image.h>
#include <sens_loc/math/pointcloud.h>
#include <sens_loc/plot/backprojection.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/thread_analysis.h>
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
    vector<cv::KeyPoint> keypoints;
    cv::Mat              descriptors;
    math::image<ushort>  depth_image;
    math::pose_t         absolute_pose;

    reprojection_data(string_view feature_path,
                      string_view depth_path,
                      string_view pose_path) noexcept(false) {
        const FileStorage fs = io::open_feature_file(string(feature_path));
        keypoints            = io::load_keypoints(fs);
        descriptors          = io::load_descriptors(fs);

        optional<math::image<ushort>> d_img =
            io::load_image<ushort>(string(depth_path), IMREAD_UNCHANGED);
        if (!d_img) {
            ostringstream oss;
            oss << "Could not load depth image from " << depth_path << "!";
            throw runtime_error{oss.str()};
        }
        depth_image = move(*d_img);

        ifstream               pose_file{string(pose_path)};
        optional<math::pose_t> pose = io::load_pose(pose_file);
        if (!pose) {
            ostringstream oss;
            oss << "Could not load pose from " << pose_path << "!";
            throw runtime_error{oss.str()};
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

struct recognition_data {
    recognition_data() = default;

    void insert_recognition(span<float>                         distances,
                            const analysis::element_categories& classification,
                            size_t masked_points) noexcept {
        lock_guard l{_mutex};

        _selected_elements_distance.insert(_selected_elements_distance.end(),
                                           begin(distances), end(distances));
        _stats.account(classification);
        _totally_masked += narrow<int>(masked_points);
    }

    tuple<vector<float>, analysis::recognition_statistic, int64_t>
    extract() noexcept {
        lock_guard l{_mutex};
        tuple<vector<float>, analysis::recognition_statistic, int64_t> t{
            move(_selected_elements_distance), move(_stats), _totally_masked};

        _selected_elements_distance = vector<float>();
        _stats                      = analysis::recognition_statistic();
        _totally_masked             = 0L;
        return t;
    }

  private:
    mutex                                     _mutex;
    vector<float> _selected_elements_distance GUARDED_BY(_mutex);
    analysis::recognition_statistic _stats    GUARDED_BY(_mutex);
    int64_t _totally_masked                   GUARDED_BY(_mutex) = 0L;
};

template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
class prec_recall_analysis {
  public:
    prec_recall_analysis(
        string_view                                      feature_file_pattern,
        const apps::recognition_analysis_input&          input,
        const apps::recognition_analysis_output_options& output_options,
        recognition_data&                                accumulated_data,
        const apps::backproject_config&                  backproject_config)
        : _feature_file_pattern{feature_file_pattern}
        , _input{input}
        , _output_options{output_options}
        , _matcher{cv::BFMatcher::create(_input.matching_norm,
                                         /*crosscheck=*/true)}
        , _mask{nullopt}
        , _accumulated_data{accumulated_data}
        , _backprojection_config{backproject_config} {
        Expects(!_feature_file_pattern.empty());
        Expects(!_input.depth_image_pattern.empty());
        Expects(!_input.pose_file_pattern.empty());
        Expects(!_input.intrinsic_file.empty());

        ifstream intrinsic{string(_input.intrinsic_file)};

        auto maybe_intrinsic =
            io::camera<Real, Model>::load_intrinsic(intrinsic);
        if (!maybe_intrinsic.has_value()) {
            stringstream ss;
            ss << "Intrinsic file " << _input.intrinsic_file
               << " could not be loaded!";
            throw std::invalid_argument{ss.str()};
        }
        _intrinsic = *maybe_intrinsic;

        if (_input.mask_file) {
            _mask = io::load_image<uchar>(string(*_input.mask_file),
                                          IMREAD_GRAYSCALE);
            if (!_mask) {
                auto s = synced();
                cerr << util::err{} << "Could not load mask file '"
                     << *_input.mask_file << "'!\n"
                     << "Continuing without masking.\n";
            }
            // The mask file is loaded. It is necessary to check, that the
            // dimensions match the loaded intrinsic. If not, masking is
            // disabled.
            else {
                if (_mask->w() != _intrinsic.w() ||
                    _mask->h() != _intrinsic.h()) {
                    auto s = synced();
                    cerr << util::err{}
                         << "Dimensions of the mask mismatch the dimensions of "
                            "the intrinsic!\n"
                         << "Continuing without masking masking.\n"
                         << "Mask {" << _mask->w() << ", " << _mask->h()
                         << "} vs Intrinsic {" << _intrinsic.w() << ", "
                         << _intrinsic.h() << "}\n";
                    // Resetting the mask to none!
                    _mask = nullopt;
                }
            }
        }

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

        reprojection_data prev{
            fmt::format(_feature_file_pattern, previous_idx),
            fmt::format(_input.depth_image_pattern, previous_idx),
            fmt::format(_input.pose_file_pattern, previous_idx)};
        reprojection_data curr{fmt::format(_feature_file_pattern, idx),
                               fmt::format(_input.depth_image_pattern, idx),
                               fmt::format(_input.pose_file_pattern, idx)};

        if (prev.keypoints.empty() || curr.keypoints.empty())
            return;

        // == Calculate relative pose between the two frames.
        pose_t rel_pose = relative_pose(prev.absolute_pose, curr.absolute_pose);

        // Refine that pose with an ICP if possible.
        if (_icp) {
            auto [icp_pose, icp_success] =
                refine_pose(*_icp, prev.depth_image, curr.depth_image,
                            _input.unit_factor, rel_pose);
            if (icp_success) {
                rel_pose = icp_pose;
            } else {
                auto s = synced();
                cerr << util::warn{} << "No ICP result for index " << idx
                     << "! Using loaded pose.\n";
            }
        }

        // == get keypoints as world points
        pointcloud_t prev_points = keypoints_to_pointcloud(
            prev.keypoints, prev.depth_image, _intrinsic, _input.unit_factor);

        imagepoints_t prev_in_img =
            project_to_other_camera(rel_pose, prev_points, _intrinsic);

        // Some backprojected points might land outside of the viewport of
        // the camera model, but still on valid pixel coordinates (inside the
        // image). This can be detected with using a mask.
        // Such points are set to {-1, -1} to be recognizable invalid.
        const size_t masked_points =
            _mask ? mask_backprojection(*_mask, prev_in_img) : 0UL;
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
            curr_keypoints, prev_in_img, matches,
            _input.keypoint_distance_threshold);
        // True positives keypoints from this and previous frame in this
        // frames coordinate system.
        auto [t_p_t, t_p_o] = analysis::gather_correspondences(
            classification.true_positives, curr_keypoints, prev_in_img);

        if (_output_options.backproject_pattern) {
            string orig_f_name =
                fmt::format(*_output_options.original_files, idx);
            optional<math::image<uchar>> orig_img =
                io::load_as_8bit_gray(orig_f_name);

            if (orig_img) {
                // Plot true positives.
                Mat out_img = plot::backprojection_correspondence(
                    orig_img->data(), t_p_t, t_p_o,  // Keypoints
                    _backprojection_config.true_positive.color,
                    _backprojection_config.true_positive.strength);

                // Plot false negatives with purple line
                auto [f_n_t, f_n_o] = analysis::gather_correspondences(
                    classification.false_negatives, curr_keypoints,
                    prev_in_img);
                out_img = plot::backprojection_correspondence(
                    out_img, f_n_t, f_n_o,  // Base image and keypoints
                    _backprojection_config.false_negative.color,
                    _backprojection_config.false_negative.strength);

                // Plot false positives with orange distance line
                auto [f_p_t, f_p_o] = analysis::gather_correspondences(
                    classification.false_positives, curr_keypoints,
                    prev_in_img);
                out_img = plot::backprojection_correspondence(
                    out_img, f_p_t, f_p_o,  // Base image and keypoints
                    _backprojection_config.false_positive.color,
                    _backprojection_config.false_positive.strength);

                const bool write_success = imwrite(
                    fmt::format(*_output_options.backproject_pattern, idx),
                    out_img);

                if (!write_success) {
                    auto s = synced();
                    cerr << util::err{}
                         << "Could not write backprojection correspondence to "
                            "disk for: "
                         << idx << "!\n";
                }
            } else {
                auto s = synced();
                cerr << util::warn{} << "Original file for index '" << idx
                     << "' ('" << orig_f_name
                     << "') could not be loaded for backprojection plot!\n";
            }
        }

        auto distances = pointwise_distance(t_p_t, t_p_o);
        _accumulated_data.insert_recognition(distances, classification,
                                             masked_points);
    } catch (const exception& e) {
        auto s = synced();
        cerr << util::err{} << "Could not analyze data for " << idx << "!\n"
             << e.what() << "\n";
        return;
    }

    size_t postprocess() {
        auto [distances, classification, masked_point_count] =
            _accumulated_data.extract();

        if (classification.total_elements() == 0L)
            return 0UL;

        sort(begin(distances), end(distances));
        const auto         dist_bins = 20;
        analysis::distance distance_stat{distances, dist_bins,
                                         "backprojection error pixels"};

        if (_output_options.stat_file) {
            cv::FileStorage recognize_out{*_output_options.stat_file,
                                          cv::FileStorage::WRITE |
                                              cv::FileStorage::FORMAT_YAML};
            write(recognize_out, "classification", classification);
            write(recognize_out, "masked_points",
                  narrow<int>(masked_point_count));
            recognize_out.release();
        } else {
            using namespace boost;
            cout << "# relevant:   " << distance_stat.count() << "\n"
                 << "min:          " << distance_stat.min() << "\n"
                 << "max:          " << distance_stat.max() << "\n"
                 << "median:       " << distance_stat.median() << "\n"
                 << "mean:         " << distance_stat.mean() << "\n"
                 << "Variance:     " << distance_stat.variance() << "\n"
                 << "StdDev:       " << distance_stat.stddev() << "\n"
                 << "Skewness:     " << distance_stat.skewness() << "\n"
                 << "Precision     " << classification.precision() << "\n"
                 << "Recall:       " << classification.recall() << "\n"
                 << "Sensitivity:  " << classification.sensitivity() << "\n"
                 << "Specificity:  " << classification.specificity() << "\n"
                 << "Fallout:      " << classification.fallout() << "\n"
                 << "Rand-Index:   " << classification.rand_index() << "\n"
                 << "Youden-Index: " << classification.youden_index() << "\n"
                 << "Min-TruePos:  "
                 << accumulators::min(
                        classification.true_positive_distribution().stat)
                 << "\n"
                 << "Avg-TruePos:  "
                 << accumulators::mean(
                        classification.true_positive_distribution().stat)
                 << "\n"
                 << "Min-Relevant: "
                 << accumulators::min(
                        classification.relevant_element_distribution().stat)
                 << "\n"
                 << "Avg-Relevant: "
                 << accumulators::mean(
                        classification.relevant_element_distribution().stat)
                 << "\n"
                 << "Avg-FalsePos: "
                 << accumulators::mean(
                        classification.false_positive_distribution().stat)
                 << "\n"
                 << "Masked pts:   " << masked_point_count << "\n";
        }

        if (_output_options.backprojection_selected_histo) {
            ofstream gnuplot_data{
                *_output_options.backprojection_selected_histo};
            gnuplot_data << sens_loc::io::to_gnuplot(distance_stat.histogram())
                         << endl;
        } else {
            cout << distance_stat.histogram() << "\n";
        }

        classification.make_histogram();

        if (_output_options.relevant_histo) {
            ofstream gnuplot_data{*_output_options.relevant_histo};
            gnuplot_data
                << sens_loc::io::to_gnuplot(
                       classification.relevant_element_distribution().histo)
                << endl;
        } else {
            cout << classification.relevant_element_distribution().histo
                 << "\n";
        }

        if (_output_options.true_positive_histo) {
            ofstream gnuplot_data{*_output_options.true_positive_histo};
            gnuplot_data
                << sens_loc::io::to_gnuplot(
                       classification.true_positive_distribution().histo)
                << endl;
        } else {
            cout << classification.true_positive_distribution().histo << "\n";
        }

        if (_output_options.false_positive_histo) {
            ofstream gnuplot_data{*_output_options.false_positive_histo};
            gnuplot_data
                << sens_loc::io::to_gnuplot(
                       classification.false_positive_distribution().histo)
                << endl;
        } else {
            cout << classification.false_positive_distribution().histo << "\n";
        }

        if (_output_options.true_positive_distance_histo) {
            ofstream gnuplot_data{
                *_output_options.true_positive_distance_histo};
            gnuplot_data
                << sens_loc::io::to_gnuplot(
                       classification.true_positive_distance().histogram())
                << endl;
        } else {
            cout << classification.true_positive_distance().histogram() << "\n";
        }

        if (_output_options.false_positive_distance_histo) {
            ofstream gnuplot_data{
                *_output_options.false_positive_distance_histo};
            gnuplot_data
                << sens_loc::io::to_gnuplot(
                       classification.false_positive_distance().histogram())
                << endl;
        } else {
            cout << classification.false_positive_distance().histogram()
                 << "\n";
        }

        return classification.total_elements();
    }

  private:
    std::string_view                                 _feature_file_pattern;
    const apps::recognition_analysis_input&          _input;
    const apps::recognition_analysis_output_options& _output_options;

    Model<Real>                  _intrinsic;
    Ptr<rgbd::Odometry>          _icp;
    Ptr<BFMatcher>               _matcher;
    optional<math::image<uchar>> _mask;
    recognition_data&            _accumulated_data;

    const apps::backproject_config& _backprojection_config;
};
}  // namespace

namespace sens_loc::apps {
int analyze_recognition_performance(
    util::processing_input                     in,
    const recognition_analysis_input&          required_data,
    const recognition_analysis_output_options& output_options,
    const backproject_config&                  backproject_config) {
    Expects(in.start < in.end &&
            "Recognition Performance calculation requires at least two images");

    // The code will load all data directly and does not rely on loading through
    // the statistics code.
    using visitor =
        statistic_visitor<prec_recall_analysis<>, required_data::none>;

    recognition_data accumulator;
    // The odd-looking double arguments comes from the genericity of the
    // statistic-visitation. The first argument goes to \c statistic_visitor
    // and the second one to \c prec_recall_analysis
    auto analysis_v =
        visitor{in.input_pattern, in.input_pattern, required_data,
                output_options,   accumulator,      backproject_config};

    // Consecutive images are matched and analysed, therefore the first
    // index must be skipped.
    auto   f          = parallel_visitation(in.start + 1, in.end, analysis_v);
    size_t n_elements = f.postprocess();

    return n_elements > 0L ? 0 : 1;
}

}  // namespace sens_loc::apps
