#include "keypoint_distribution.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <sstream>
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

namespace {

template <typename Histogram2D>
std::string histogram_to_gnuplot(const Histogram2D& h) noexcept {
    std::ostringstream os;

    // The second index is running slow, so that needs to be check for
    // line separation, which is expected by gnuplot.
    int previous_index = 0;
    for (auto&& cell : boost::histogram::indexed(h)) {
        // Add a additional new line if there was a jump in the slow running
        // index.
        os << (cell.index(1) > previous_index ? "\n" : "") << cell.index(0)
           << " " << cell.index(1) << " " << *cell << "\n";
        previous_index = cell.index(1);
    }

    return os.str();
}

/// Calculate the 2-dimensional distribution of the keypoints for a dataset.
class keypoint_distribution {
  public:
    keypoint_distribution(gsl::not_null<std::mutex*> points_mutex,
                          gsl::not_null<std::vector<cv::KeyPoint>*> points,
                          gsl::not_null<std::mutex*>         distance_mutex,
                          gsl::not_null<std::vector<float>*> distances) noexcept
        : keypoint_mutex{points_mutex}
        , global_keypoints{points}
        , distance_mutex{distance_mutex}
        , global_distances{distances} {}

    void
    operator()(int /*idx*/,
               std::optional<std::vector<cv::KeyPoint>> keypoints,
               std::optional<cv::Mat> /*descriptors*/) noexcept {  // NOLINT
        if (keypoints->empty())
            return;

        // Simply insert the keypoints in the global keypoint vector.
        {
            std::lock_guard guard{*keypoint_mutex};
            global_keypoints->insert(std::end(*global_keypoints),
                                     std::begin(*keypoints),
                                     std::end(*keypoints));
        }

        // Calculate the minimal distance of each keypoint to all others
        // and insert that first into a local vector with that information
        // and finally into the global vector with that information.
        // This is the pixel-distance with euclidean norm.
        // NOTE: This is an inefficient implementation if O(n^2) complexity.
        {
            const std::size_t n_points = keypoints->size();
            if (n_points < 2)
                return;

            std::vector<cv::KeyPoint>& kp_ref = *keypoints;
            std::vector<float>         local_minima;
            std::vector<float>         local_distances;
            local_distances.reserve(n_points);

            // The upper triangle of the distance matrix needs to be calculated.
            // The last row must be ignored and the diagonal element will be 0.
            // Example:
            //    ++p1++p2++p3++
            // p1 |  0   2   4 |
            // p2 |      0   7 |
            // p3 |          0 |
            // The result are 'n_points / 2' number of minimal distances.
            // Because there will
            for (std::size_t i = 1; i < n_points - 1; ++i) {
                // loop-calculation
                for (std::size_t k = i + 1; k < n_points; ++k) {
                    const float dx = kp_ref[i].pt.x - kp_ref[k].pt.x;
                    const float dy = kp_ref[i].pt.y - kp_ref[k].pt.y;
                    const float d  = std::sqrt(dx * dx + dy * dy);
                    Ensures(d >= 0.0F);
                    local_distances.emplace_back(d);
                }

                Ensures(!local_distances.empty());

                // Store the minimal element for statistical evaluation.
                local_minima.emplace_back(*std::min_element(
                    std::begin(local_distances), std::end(local_distances)));

                // Ensure that local_distances is only allocated once.
                local_distances.clear();
            }

            // Insertion
            {
                std::lock_guard guard{*distance_mutex};
                global_distances->insert(global_distances->end(),
                                         std::begin(local_minima),
                                         std::end(local_minima));
            }
        }
    }

    auto postprocess(int image_width, int image_height) {
        Expects(!global_keypoints->empty());
        Expects(!global_distances->empty());

        using namespace boost::histogram;
        // Number of bins in one direction for 2D histogram.
        auto location_bins  = 200;
        auto location_histo = make_histogram(
            axis::regular(location_bins, 0.0F, 1.0F, "normalized width"),
            axis::regular(location_bins, 0.0F, 1.0F, "normalized height"));

        using namespace boost::accumulators;
        accumulator_set<float,
                        stats<tag::count, tag::min, tag::max, tag::median,
                              tag::mean, tag::variance(lazy)>>
            response_stat;
        accumulator_set<float,
                        stats<tag::count, tag::min, tag::max, tag::median,
                              tag::mean, tag::variance(lazy)>>
            size_stat;

        std::for_each(global_keypoints->begin(), global_keypoints->end(),
                      [&](const cv::KeyPoint& kp) {
                          response_stat(kp.response);
                          size_stat(kp.size);
                          location_histo(
                              kp.pt.x / static_cast<float>(image_width),
                              kp.pt.y / static_cast<float>(image_height));
                      });

        auto response_bins  = 50;
        auto response_range = max(response_stat) - min(response_stat);
        auto response_histo = make_histogram(axis::regular(
            response_bins, min(response_stat) - response_range / response_bins,
            max(response_stat) + response_range / response_bins, "reponse"));

        auto size_bins  = 50;
        auto size_histo = make_histogram(
            axis::regular(size_bins, 0.0F, max(size_stat) + 0.01F, "size"));

        std::for_each(global_keypoints->begin(), global_keypoints->end(),
                      [&response_histo, &size_histo](const cv::KeyPoint& kp) {
                          response_histo(kp.response);
                          size_histo(kp.size);
                      });


        accumulator_set<float,
                        stats<tag::count, tag::min, tag::max, tag::median,
                              tag::mean, tag::variance(lazy)>>
            dist_stat;
        std::for_each(global_distances->begin(), global_distances->end(),
                      [&](float d) { dist_stat(d); });
        auto dist_bins  = 50;
        auto dist_histo = make_histogram(
            axis::regular(dist_bins, min(dist_stat) - 0.01F,
                          max(dist_stat) + 0.01F, "minimal distance"));
        dist_histo.fill(*global_distances);

        std::cout << "==== Response\n"
                  << "count:  " << count(response_stat) << "\n"
                  << "min:    " << min(response_stat) << "\n"
                  << "max:    " << max(response_stat) << "\n"
                  << "median: " << median(response_stat) << "\n"
                  << "mean:   " << mean(response_stat) << "\n"
                  << response_histo << "\n"
                  << "==== Size\n"
                  << "count:  " << count(size_stat) << "\n"
                  << "min:    " << min(size_stat) << "\n"
                  << "max:    " << max(size_stat) << "\n"
                  << "median: " << median(size_stat) << "\n"
                  << "mean:   " << mean(size_stat) << "\n"
                  << size_histo << "\n"
                  << "=== Distance\n"
                  << "count:  " << count(dist_stat) << "\n"
                  << "min:    " << min(dist_stat) << "\n"
                  << "max:    " << max(dist_stat) << "\n"
                  << "median: " << median(dist_stat) << "\n"
                  << "mean:   " << mean(dist_stat) << "\n"
                  << dist_histo << "\n";

        std::ofstream gnuplot_data{"location_histo.data"};
        gnuplot_data << histogram_to_gnuplot(location_histo) << std::endl;
    }

  private:
    // Data required for the parallel processing.
    gsl::not_null<std::mutex*>                keypoint_mutex;
    gsl::not_null<std::vector<cv::KeyPoint>*> global_keypoints;

    gsl::not_null<std::mutex*>         distance_mutex;
    gsl::not_null<std::vector<float>*> global_distances;
};

}  // namespace

namespace sens_loc::apps {
int analyze_keypoint_distribution(std::string_view input_pattern,
                                  int              start_idx,
                                  int              end_idx,
                                  int              image_width,
                                  int              image_height) {
    using visitor =
        statistic_visitor<keypoint_distribution, required_data::keypoints>;

    std::mutex                keypoint_mutex;
    std::vector<cv::KeyPoint> global_keypoints;

    std::mutex         distance_mutex;
    std::vector<float> global_minimal_distances;

    auto f = parallel_visitation(
        start_idx, end_idx,
        visitor{input_pattern, gsl::not_null{&keypoint_mutex},
                gsl::not_null{&global_keypoints},
                gsl::not_null{&distance_mutex},
                gsl::not_null{&global_minimal_distances}});

    f.postprocess(image_width, image_height);

    return 0;
}
}  // namespace sens_loc::apps
