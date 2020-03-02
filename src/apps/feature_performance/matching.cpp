#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
#include "matching.h"

#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <fstream>
#include <gsl/gsl>
#include <iterator>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/io/histogram.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/thread_analysis.h>
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

using namespace cv;
using namespace std;
using namespace gsl;

namespace {

struct descriptor_stat_data {
    descriptor_stat_data() = default;

    void insert_matches(gsl::span<DMatch> matches,
                        int               descriptor_count) noexcept {
        lock_guard l{_mutex};
        transform(begin(matches), end(matches),
                  back_inserter(_global_minimal_distances),
                  [](const DMatch& m) { return m.distance; });
        _total_descriptors += descriptor_count;
    }

    pair<vector<float>, int64_t> extract() noexcept {
        lock_guard                   l{_mutex};
        pair<vector<float>, int64_t> p{move(_global_minimal_distances),
                                       _total_descriptors};
        _global_minimal_distances = vector<float>();
        _total_descriptors        = 0UL;
        return p;
    }

  private:
    mutex                                   _mutex;
    vector<float> _global_minimal_distances GUARDED_BY(_mutex);
    int64_t _total_descriptors              GUARDED_BY(_mutex) = 0L;
};

class matching {
  public:
    matching(descriptor_stat_data& accumulated_data,
             NormTypes             norm_to_use,
             bool                  crosscheck,
             string_view           input_pattern,
             optional<string_view> output_pattern,
             optional<string_view> original_files) noexcept
        : accumulated_data{accumulated_data}
        , matcher{BFMatcher::create(norm_to_use, crosscheck)}
        , input_pattern{input_pattern}
        , output_pattern{output_pattern}
        , original_images{original_files} {
        // XOR is true if both operands have the same value.
        Expects(!(output_pattern.has_value() ^ original_images.has_value()) &&
                "Either both or none are set");
    }

    void operator()(int idx,
                    optional<vector<KeyPoint>> /*keypoints*/,  // NOLINT
                    optional<Mat> descriptors) noexcept {
        Expects(descriptors.has_value());
        if (descriptors->rows == 0)
            return;

        try {
            const int         previous_idx = idx - 1;
            const FileStorage previous_img = sens_loc::io::open_feature_file(
                fmt::format(input_pattern, previous_idx));
            Mat previous_descriptors =
                sens_loc::io::load_descriptors(previous_img);

            vector<DMatch> matches;
            matcher->match(*descriptors, previous_descriptors, matches);
            accumulated_data.insert_matches(matches, descriptors->rows);

            // Plot the matching between the descriptors of the previous and the
            // current frame.
            if (output_pattern) {
                const vector<KeyPoint> previous_keypoints =
                    sens_loc::io::load_keypoints(previous_img);

                const FileStorage this_feature =
                    sens_loc::io::open_feature_file(
                        fmt::format(input_pattern, idx));
                const vector<KeyPoint> this_keypoints =
                    sens_loc::io::load_keypoints(this_feature);

                const string img_p1 = fmt::format(*original_images, idx - 1);
                const string img_p2 = fmt::format(*original_images, idx);
                auto         img1   = sens_loc::io::load_as_8bit_gray(img_p1);
                auto         img2   = sens_loc::io::load_as_8bit_gray(img_p2);

                if (!img1 || !img2)
                    return;

                Mat out_img;
                drawMatches(img2->data(), this_keypoints, img1->data(),
                            previous_keypoints, matches, out_img,
                            Scalar(0, 0, 255), Scalar(255, 0, 0));

                const string output = fmt::format(*output_pattern, idx);
                imwrite(output, out_img);
            }
        } catch (...) {
            std::cerr << sens_loc::util::err{}
                      << "Could not initialize data for idx: " << idx << "\n";
            return;
        }
    }

    size_t postprocess(const optional<string>& stat_file,
                       const optional<string>& matched_distance_histo) {
        auto [distances, total_descriptors] = accumulated_data.extract();
        if (distances.empty())
            return 0UL;

        sort(begin(distances), end(distances));
        const auto                   dist_bins = 25;
        sens_loc::analysis::distance distance_stat{distances, dist_bins};

        if (stat_file) {
            cv::FileStorage stat_out{*stat_file,
                                     cv::FileStorage::WRITE |
                                         cv::FileStorage::FORMAT_YAML};
            stat_out.writeComment(
                "The following values contain the results of the statistical "
                "analysis for descriptor distance to the closest descriptor "
                "after matching");
            write(stat_out, "match_distance", distance_stat.get_statistic());
            stat_out.release();
        } else {
            cout << "==== Match Distances\n"
                 << "total count:    " << total_descriptors << "\n"
                 << "matched count:  " << distances.size() << "\n"
                 << "matched/total:  "
                 << narrow_cast<double>(distances.size()) /
                        narrow_cast<double>(total_descriptors)
                 << "\n"
                 << "min:            " << distance_stat.min() << "\n"
                 << "max:            " << distance_stat.max() << "\n"
                 << "median:         " << distance_stat.median() << "\n"
                 << "mean:           " << distance_stat.mean() << "\n"
                 << "Variance:       " << distance_stat.variance() << "\n"
                 << "StdDev:         " << distance_stat.stddev() << "\n"
                 << "Skewness:       " << distance_stat.skewness() << "\n";
        }
        if (matched_distance_histo) {
            std::ofstream gnuplot_data{*matched_distance_histo};
            gnuplot_data << sens_loc::io::to_gnuplot(distance_stat.histogram())
                         << std::endl;
        } else {
            cout << distance_stat.histogram() << "\n";
        }
        return distances.size();
    }

  private:
    descriptor_stat_data& accumulated_data;

    Ptr<BFMatcher>        matcher;
    string_view           input_pattern;
    optional<string_view> output_pattern;
    optional<string_view> original_images;
};
}  // namespace

namespace sens_loc::apps {
int analyze_matching(util::processing_input       in,
                     NormTypes                    norm_to_use,
                     bool                         crosscheck,
                     const optional<string>&      stat_file,
                     const optional<string>&      matched_distance_histo,
                     const optional<string_view>& output_pattern,
                     const optional<string_view>& original_files) {
    Expects(in.start < in.end && "Matching requires at least 2 images");
    using visitor = statistic_visitor<matching, required_data::descriptors>;
    descriptor_stat_data data;
    auto analysis_v = visitor{/*input_pattern=*/in.input_pattern,
                              /*accumulated_data=*/data,
                              /*norm_to_use=*/norm_to_use,
                              /*crosscheck=*/crosscheck,
                              /*input_pattern=*/in.input_pattern,
                              /*output_pattern=*/output_pattern,
                              /*original_files=*/original_files};

    auto f = parallel_visitation(
        in.start + 1,  // Because two consecutive images are matched, the first
                       // index is skipped. This requires "backwards" matching.
        in.end, analysis_v);
    size_t n_elements = f.postprocess(stat_file, matched_distance_histo);

    return n_elements > 0UL ? 0 : 1;
}
}  // namespace sens_loc::apps
