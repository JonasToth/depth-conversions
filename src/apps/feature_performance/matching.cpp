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
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

namespace {
class matching {
  public:
    matching(gsl::not_null<std::mutex*>         distance_mutex,
             gsl::not_null<std::vector<float>*> distances,
             gsl::not_null<std::uint64_t*>      descriptor_count,
             cv::NormTypes                      norm_to_use,
             bool                               crosscheck,
             std::string_view                   input_pattern) noexcept
        : distance_mutex{distance_mutex}
        , distances{distances}
        , total_descriptors{descriptor_count}
        , matcher{cv::BFMatcher::create(norm_to_use, crosscheck)}
        , input_pattern{input_pattern} {}

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
};
}  // namespace

namespace sens_loc::apps {
int analyze_matching(std::string_view input_pattern,
                     int              start_idx,
                     int              end_idx,
                     cv::NormTypes    norm_to_use,
                     bool             crosscheck) {
    Expects(start_idx < end_idx && "Matching requires at least 2 images");

    using visitor = statistic_visitor<matching, required_data::descriptors>;

    std::mutex         distance_mutex;
    std::vector<float> global_minimal_distances;
    std::uint64_t      total_descriptors;

    auto f = parallel_visitation(
        start_idx + 1,  // Because two consecutive images are matched, the first
                        // index is skipped. This requires "backwards" matching.
        end_idx,
        visitor{input_pattern, gsl::not_null{&distance_mutex},
                gsl::not_null{&global_minimal_distances},
                gsl::not_null{&total_descriptors}, norm_to_use, crosscheck,
                input_pattern});

    f.postprocess();

    return 0;
}
}  // namespace sens_loc::apps
