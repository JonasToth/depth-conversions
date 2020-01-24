#include "matching.h"

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
#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <gsl/gsl>
#include <iterator>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <util/batch_visitor.h>
#include <util/io.h>
#include <util/statistic_visitor.h>

namespace {
class matching {
  public:
    matching(gsl::not_null<std::mutex*>         distance_mutex,
             gsl::not_null<std::vector<float>*> distances,
             gsl::not_null<std::uint64_t*>      descriptor_count,
             cv::NormTypes                      norm_to_use,
             bool                               crosscheck,
             std::string_view                   input_pattern,
             std::optional<std::string_view>    output_pattern,
             std::optional<std::string_view>    original_files) noexcept
        : distance_mutex{distance_mutex}
        , distances{distances}
        , total_descriptors{descriptor_count}
        , matcher{cv::BFMatcher::create(norm_to_use, crosscheck)}
        , input_pattern{input_pattern}
        , output_pattern{output_pattern}
        , original_images{original_files} {
        // XOR is true if both operands have the same value.
        Expects(!(output_pattern.has_value() ^ original_images.has_value()) &&
                "Either both or none are set");
    }

    void operator()(
        int idx,
        std::optional<std::vector<cv::KeyPoint>> /*keypoints*/,  // NOLINT
        std::optional<cv::Mat> descriptors) noexcept {
        Expects(descriptors);
        Expects(descriptors->rows > 0);

        const int             previous_idx = idx - 1;
        const cv::FileStorage previous_img =
            sens_loc::apps::open_feature_file(input_pattern, previous_idx);
        cv::Mat previous_descriptors =
            sens_loc::apps::load_descriptors(previous_img);

        std::vector<cv::DMatch> matches;
        matcher->match(*descriptors, previous_descriptors, matches);

        {
            std::lock_guard guard{*distance_mutex};
            // Insert all the distances into the global distances vector.
            std::transform(std::begin(matches), std::end(matches),
                           std::back_inserter(*distances),
                           [](const cv::DMatch& m) { return m.distance; });
            (*total_descriptors) += descriptors->rows;
        }

        if (output_pattern) {
            const cv::FileStorage this_feature =
                sens_loc::apps::open_feature_file(input_pattern, idx);
            const std::vector<cv::KeyPoint> previous_keypoints =
                sens_loc::apps::load_keypoints(this_feature);

            const std::vector<cv::KeyPoint> this_keypoints =
                sens_loc::apps::load_keypoints(previous_img);

            const std::string img_p1 = fmt::format(*original_images, idx - 1);
            const std::string img_p2 = fmt::format(*original_images, idx);
            auto              img1   = sens_loc::apps::load_file(img_p1);
            auto              img2   = sens_loc::apps::load_file(img_p2);

            if (!img1 || !img2)
                return;

            cv::Mat out_img;
            cv::drawMatches(img1->data(), previous_keypoints, img2->data(),
                            this_keypoints, matches, out_img,
                            cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0));

            const std::string output = fmt::format(*output_pattern, idx);
            cv::imwrite(output, out_img);
        }
    }

    void postprocess() {
        std::lock_guard guard{*distance_mutex};
        Expects(!distances->empty());

        using namespace boost::accumulators;
        accumulator_set<float,
                        stats<tag::count, tag::min, tag::max, tag::median,
                              tag::mean, tag::variance(lazy), tag::skewness>>
            distance_stat;
        std::for_each(distances->begin(), distances->end(),
                      [&](float d) { distance_stat(d); });

        using namespace boost::histogram;
        auto dist_bins  = 25;
        auto dist_histo = make_histogram(
            axis::regular(dist_bins, min(distance_stat) - 0.01F,
                          max(distance_stat) + 0.01F, "match distance"));
        dist_histo.fill(*distances);

        std::cout << "==== Match Distances\n"
                  << dist_histo << "\n"
                  << "total count:    " << *total_descriptors << "\n"
                  << "matched count:  " << count(distance_stat) << "\n"
                  << "matched/total:  "
                  << static_cast<double>(count(distance_stat)) /
                         static_cast<double>(*total_descriptors)
                  << "\n"
                  << "min:            " << min(distance_stat) << "\n"
                  << "max:            " << max(distance_stat) << "\n"
                  << "median:         " << median(distance_stat) << "\n"
                  << "mean:           " << mean(distance_stat) << "\n"
                  << "Variance:       " << variance(distance_stat) << "\n"
                  << "Skewness:       " << skewness(distance_stat) << "\n";
    }

  private:
    gsl::not_null<std::mutex*>         distance_mutex;
    gsl::not_null<std::vector<float>*> distances;
    gsl::not_null<std::uint64_t*>      total_descriptors;
    cv::Ptr<cv::BFMatcher>             matcher;
    std::string_view                   input_pattern;
    std::optional<std::string_view>    output_pattern;
    std::optional<std::string_view>    original_images;
};
}  // namespace

namespace sens_loc::apps {
int analyze_matching(std::string_view                input_pattern,
                     int                             start_idx,
                     int                             end_idx,
                     cv::NormTypes                   norm_to_use,
                     bool                            crosscheck,
                     std::optional<std::string_view> output_pattern,
                     std::optional<std::string_view> original_files) {
    Expects(start_idx < end_idx && "Matching requires at least 2 images");

    std::mutex         distance_mutex;
    std::vector<float> global_minimal_distances;
    std::uint64_t      total_descriptors;

    using visitor = statistic_visitor<matching, required_data::descriptors>;
    auto analysis_v =
        visitor{/*input_pattern=*/input_pattern,
                /*distance_mutex=*/gsl::not_null{&distance_mutex},
                /*distances=*/gsl::not_null{&global_minimal_distances},
                /*descriptor_count=*/gsl::not_null{&total_descriptors},
                /*norm_to_use=*/norm_to_use,
                /*crosscheck=*/crosscheck,
                /*input_pattern=*/input_pattern,
                /*output_pattern=*/output_pattern,
                /*original_files=*/original_files};

    auto f = parallel_visitation(
        start_idx + 1,  // Because two consecutive images are matched, the first
                        // index is skipped. This requires "backwards" matching.
        end_idx, analysis_v);

    f.postprocess();

    return 0;
}
}  // namespace sens_loc::apps
