#include "keypoint_distribution.h"

#include <boost/histogram/ostream.hpp>
#include <fstream>
#include <gsl/gsl>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/analysis/keypoints.h>
#include <sens_loc/io/histogram.h>
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

namespace {

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

    void postprocess(unsigned int image_width, unsigned int image_height) {
        if (global_keypoints->empty() || global_distances->empty())
            return;

        sens_loc::analysis::keypoints kp{image_width, image_height};

        const auto location_bins = 200U;
        kp.configure_distribution(location_bins);
        kp.configure_distribution("normalized width", "normalized height");

        const auto response_bins = 50U;
        kp.configure_response(response_bins, "detector response");

        const auto size_bins = 50U;
        kp.configure_size(size_bins, "keypoint size");
        kp.analyze(*global_keypoints);

        const auto                   dist_bins = 50UL;
        sens_loc::analysis::distance distance_stat{*global_distances, dist_bins,
                                                   "minimal keypoint distance"};

        cv::FileStorage kp_statistic{"keypoint.stat",
                                     cv::FileStorage::WRITE |
                                         cv::FileStorage::FORMAT_YAML};
        kp_statistic.writeComment(
            "The following values contain the results of the statistical "
            "analysis for the keypoint distribution and detector results.");
        write(kp_statistic, "characteristics", kp);
        write(kp_statistic, "distance", distance_stat.get_statistic());
        kp_statistic.release();

        std::cout << "==== Response\n"
                  << "count:  " << kp.response().count << "\n"
                  << "min:    " << kp.response().min << "\n"
                  << "max:    " << kp.response().max << "\n"
                  << "median: " << kp.response().median << "\n"
                  << "mean:   " << kp.response().mean << "\n"
                  << "var:    " << kp.response().variance << "\n"
                  << "stddev: " << kp.response().stddev << "\n"
                  << kp.response_histo() << "\n"

                  << "==== Size\n"
                  << "count:  " << kp.size().count << "\n"
                  << "min:    " << kp.size().min << "\n"
                  << "max:    " << kp.size().max << "\n"
                  << "median: " << kp.size().median << "\n"
                  << "mean:   " << kp.size().mean << "\n"
                  << "var:    " << kp.size().variance << "\n"
                  << "stddev: " << kp.size().stddev << "\n"
                  << kp.size_histo() << "\n"

                  << "=== Distance\n"
                  << "count:  " << distance_stat.count() << "\n"
                  << "min:    " << distance_stat.min() << "\n"
                  << "max:    " << distance_stat.max() << "\n"
                  << "median: " << distance_stat.median() << "\n"
                  << "mean:   " << distance_stat.mean() << "\n"
                  << "stddev: " << distance_stat.stddev() << "\n"
                  << distance_stat.histogram() << "\n";

        std::ofstream gnuplot_data{"location_histo.data"};
        gnuplot_data << sens_loc::io::to_gnuplot(kp.distribution())
                     << std::endl;
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
                                  unsigned int     image_width,
                                  unsigned int     image_height) {
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

    return !global_minimal_distances.empty() ? 0 : 1;
}
}  // namespace sens_loc::apps
