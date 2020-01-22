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
#include <boost/histogram/make_histogram.hpp>
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
    keypoint_distribution(
        gsl::not_null<std::mutex*>                m,
        gsl::not_null<std::vector<cv::KeyPoint>*> points) noexcept
        : process_mutex{m}
        , global_keypoints{points} {}

    void operator()(int /*idx*/,
                    std::optional<std::vector<cv::KeyPoint>> keypoints,
                    std::optional<cv::Mat> /*descriptors*/) noexcept {
        std::lock_guard guard{*process_mutex};
        global_keypoints->insert(std::end(*global_keypoints),
                                 std::begin(*keypoints), std::end(*keypoints));
    }

    auto postprocess(int image_width, int image_height) {
        std::lock_guard guard{*process_mutex};
        Expects(!global_keypoints->empty());

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
                  << size_histo << std::endl;

        std::ofstream gnuplot_data{"location_histo.data"};
        gnuplot_data << histogram_to_gnuplot(location_histo) << std::endl;
    }

  private:
    // Data required for the parallel processing.
    gsl::not_null<std::mutex*>                process_mutex;
    gsl::not_null<std::vector<cv::KeyPoint>*> global_keypoints;
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

    std::mutex                process_mutex;
    std::vector<cv::KeyPoint> global_keypoints;

    auto f = parallel_visitation(start_idx, end_idx,
                                 visitor{input_pattern,
                                         gsl::not_null{&process_mutex},
                                         gsl::not_null{&global_keypoints}});

    f.postprocess(image_width, image_height);

    return 0;
}
}  // namespace sens_loc::apps
