#include "precision_recall.h"

#include "gsl/gsl_assert"
#include "sens_loc/math/coordinate.h"

#include <Eigen/src/Geometry/Transform.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <boost/histogram/make_histogram.hpp>
#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/io/pose.h>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

namespace {
using pointcloud = Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor>;

class prec_recall_analysis {
  public:
    prec_recall_analysis(
        std::string_view                   feature_file_pattern,
        std::string_view                   depth_image_pattern,
        std::string_view                   pose_file_pattern,
        std::string_view                   intrinsic_file,
        cv::NormTypes                      matching_norm,
        gsl::not_null<std::mutex*>         distance_mutex,
        gsl::not_null<std::vector<float>*> global_distances) noexcept
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

        std::ifstream intrinsic{std::string(intrinsic_file)};

        auto maybe_intrinsic = sens_loc::io::camera<
            float, sens_loc::camera_models::pinhole>::load_intrinsic(intrinsic);
        Expects(maybe_intrinsic.has_value());
        _intrinsic = *maybe_intrinsic;
    }

    void operator()(int                                      idx,
                    std::optional<std::vector<cv::KeyPoint>> keypoints,
                    std::optional<cv::Mat> descriptors) noexcept {
        Expects(keypoints);
        Expects(!keypoints->empty());
        Expects(descriptors);
        Expects(std::size_t(descriptors->rows) == keypoints->size());

        const int previous_idx = idx - 1;

        using namespace sens_loc;
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
            std::cerr << "No previous depth image\n";
            return;
        }

        const string previous_pose_path =
            fmt::format(_pose_file_pattern, previous_idx);
        ifstream           previous_pose_file(previous_pose_path);
        optional<Affine3f> previous_pose = io::load_pose(previous_pose_file);
        if (!previous_pose) {
            std::cerr << "No previous pose - " << previous_pose_path << "\n";
            return;
        }

        // == Load the missing data for the current index.
        const string depth_path = fmt::format(_depth_image_pattern, idx);
        optional<math::image<ushort>> depth_image =
            io::load_image<ushort>(depth_path, cv::IMREAD_UNCHANGED);
        if (!depth_image) {
            std::cerr << "No depth image\n";
            return;
        }

        const string       pose_path = fmt::format(_pose_file_pattern, idx);
        ifstream           pose_file(pose_path);
        optional<Affine3f> pose = io::load_pose(pose_file);
        if (!pose) {
            std::cerr << "No pose - " << pose_path << "\n";
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
        pointcloud previous_points(4, matches.size());
        Ensures(size_t(previous_points.cols()) == matches.size());
        Ensures(previous_points.rows() == 4);
        previous_points.row(3).fill(1.0F);

        // Querypointcloud: this
        pointcloud points(4, matches.size());
        Ensures(size_t(points.cols()) == matches.size());
        Ensures(points.rows() == 4);
        points.row(3).fill(1.0F);

        size_t match_idx = 0;
        for (const DMatch& match : matches) {
            auto q_kp = (*keypoints)[match.queryIdx];
            auto t_kp = previous_keypoints[match.trainIdx];

            auto q_px = math::pixel_coord<float>{q_kp.pt.x, q_kp.pt.y};
            auto q_d  = float(depth_image->at(q_px));
            auto t_px = math::pixel_coord<float>{t_kp.pt.x, q_kp.pt.y};
            auto t_d  = float(previous_depth_image->at(t_px));

            auto q_s = _intrinsic.pixel_to_sphere(q_px);
            auto t_s = _intrinsic.pixel_to_sphere(t_px);

            auto q_h =
                Vector4f{q_d * q_s.Xs(), q_d * q_s.Ys(), q_d * q_s.Zs(), 1.F};
            points.col(match_idx) = q_h;

            auto t_h =
                Vector4f{t_d * t_s.Xs(), t_d * t_s.Ys(), t_d * t_s.Zs(), 1.F};
            previous_points.col(match_idx) = t_h;

            ++match_idx;
        }

        // == Calculate relative pose between the two frames.
        Affine3f relative_pose = previous_pose->inverse() * *pose;

        // == Transform matches from previous frame into this coordinate frame.
        pointcloud prev_in_this_frame = relative_pose * previous_points;

        // == Calculate the distance of each match.
        pointcloud  diff      = points - prev_in_this_frame;
        RowVectorXf distances = diff.colwise().norm();

        Ensures(distances.rows() == 1);
        Ensures(size_t(distances.cols()) == matches.size());

#if 0
        RowVectorXf::Index min_row;
        RowVectorXf::Index min_col;
        std::cout << "Min: " << distances.minCoeff(&min_row, &min_col)
                  << "  at pixel " << (*keypoints)[min_col].pt << "\n";
#endif

        {
            std::lock_guard guard{*_distance_mutex};
            for (int i = 0; i < distances.cols(); ++i)
                _global_distances->emplace_back(distances(i));
        }
    }

    void postprocess() noexcept {
        std::lock_guard guard{*_distance_mutex};

        using namespace boost::accumulators;
        accumulator_set<float,
                        stats<tag::count, tag::min, tag::max, tag::median,
                              tag::mean, tag::variance(lazy), tag::skewness>>
            distance_stat;
        std::for_each(_global_distances->begin(), _global_distances->end(),
                      [&](float d) { distance_stat(d); });

        const float med = median(distance_stat);
        auto        new_end =
            std::remove_if(_global_distances->begin(), _global_distances->end(),
                           [=](float d) { return d > med; });
        _global_distances->erase(new_end, _global_distances->end());

        using namespace boost::histogram;
        const auto dist_bins      = 50;
        auto       distance_histo = make_histogram(
            axis::regular(dist_bins, min(distance_stat), median(distance_stat),
                          "world distance of matched points"));
        distance_histo.fill(*_global_distances);
        std::cout << distance_histo << "\n"
                  << "matched count:  " << count(distance_stat) << "\n"
                  << "min:            " << min(distance_stat) << "\n"
                  << "max:            " << max(distance_stat) << "\n"
                  << "median:         " << median(distance_stat) << "\n"
                  << "mean:           " << mean(distance_stat) << "\n"
                  << "Variance:       " << variance(distance_stat) << "\n"
                  << "Skewness:       " << skewness(distance_stat) << "\n";
        _global_distances->erase(new_end, _global_distances->end());
    }

  private:
    std::string_view                        _feature_file_pattern;
    std::string_view                        _depth_image_pattern;
    std::string_view                        _pose_file_pattern;
    sens_loc::camera_models::pinhole<float> _intrinsic;
    cv::Ptr<cv::BFMatcher>                  _matcher;

    gsl::not_null<std::mutex*>         _distance_mutex;
    gsl::not_null<std::vector<float>*> _global_distances;
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
    std::mutex         distance_mutex;
    std::vector<float> global_distances;

    auto analysis_v = visitor{feature_file_pattern,
                              feature_file_pattern,
                              depth_image_pattern,
                              pose_file_pattern,
                              intrinsic_file,
                              matching_norm,
                              gsl::not_null{&distance_mutex},
                              gsl::not_null{&global_distances}};

    // Consecutive images are matched and analysed, therefore the first index
    // must be skipped.
    auto f = parallel_visitation(start_idx + 1, end_idx, analysis_v);
    f.postprocess();

    return 0;
}

}  // namespace sens_loc::apps
