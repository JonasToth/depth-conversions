#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <iostream>
#include <sens_loc/analysis/keypoints.h>
#include <sens_loc/util/console.h>
#include <stdexcept>

namespace sens_loc::analysis {

void keypoints::analyze(gsl::span<const cv::KeyPoint> points,
                        const bool                    distribution,
                        const bool                    size,
                        const bool                    response) noexcept {
    _size_histo.reset();
    _size.reset();
    _response_histo.reset();
    _response_histo.reset();
    _distribution.reset();

    Expects(!points.empty());

    using namespace boost::accumulators;
    accumulator_set<float, stats<tag::count, tag::min, tag::max, tag::median,
                                 tag::mean, tag::variance(lazy)>>
        response_stat;
    accumulator_set<float, stats<tag::count, tag::min, tag::max, tag::median,
                                 tag::mean, tag::variance(lazy)>>
        size_stat;

    try {
        std::for_each(points.begin(), points.end(),
                      [&](const cv::KeyPoint& kp) -> void {
                          if (size)
                              size_stat(kp.size);
                          if (response)
                              response_stat(kp.response);
                      });
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create statistics for size and response.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }

    using namespace boost::histogram;

    // Shortcut if no histogram will be created.
    if (!distribution || !response || !size)
        return;

    if (distribution)
        try {
            _distribution = make_histogram(
                axis_t{_dist_width_bins, 0.0F, 1.0F, _dist_w_title},
                axis_t{_dist_height_bins, 0.0F, 1.0F, _dist_h_title});
        } catch (const std::invalid_argument& e) {
            std::cerr << sens_loc::util::err{}
                      << "Bad distribution histogram configuration!\n"
                      << "Message: " << e.what() << "\n";
        }

    const float delta = 5.0F * std::numeric_limits<float>::epsilon();
    if (size && _size_histo_enabled) {
        try {
            const float s_min = min(size_stat) - delta;
            const float s_max = max(size_stat) + delta;
            _size_histo =
                make_histogram(axis_t{_size_bins, s_min, s_max, _size_title});
        } catch (const std::invalid_argument& e) {
            std::cerr
                << sens_loc::util::err{}
                << "Could not create distribution histogram configuration!\n"
                << "Message: " << e.what() << "\n";
        }
    }

    if (response && _response_histo_enabled) {
        try {
            const float r_min = min(response_stat) - delta;
            const float r_max = max(response_stat) + delta;
            _response_histo   = make_histogram(
                axis_t{_response_bins, r_min, r_max, _response_title});
        } catch (const std::invalid_argument& e) {
            std::cerr << sens_loc::util::err{}
                      << "Could not create response histogram configuration!\n"
                      << "Message: " << e.what() << "\n";
        }
    }

    try {
        std::for_each(
            points.begin(), points.end(), [&](const cv::KeyPoint& kp) -> void {
                if (distribution)
                    _distribution(kp.pt.x / static_cast<float>(_img_width),
                                  kp.pt.y / static_cast<float>(_img_height));
                if (response && _response_histo_enabled)
                    _response_histo(kp.response);
                if (size && _size_histo_enabled)
                    _size_histo(kp.size);
            });
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create histograms for size and/or response "
                     "and/or keypoint-distribution.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }
}
}  // namespace sens_loc::analysis
