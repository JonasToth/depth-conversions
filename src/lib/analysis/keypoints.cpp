#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <sens_loc/analysis/keypoints.h>

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

    std::for_each(points.begin(), points.end(),
                  [&](const cv::KeyPoint& kp) noexcept -> void {
                      if (size)
                          size_stat(kp.size);
                      if (response)
                          response_stat(kp.response);
                  });

    using namespace boost::histogram;

    // Shortcut if no histogram will be created.
    if (!distribution || !response || !size)
        return;

    if (distribution)
        _distribution = make_histogram(
            axis::regular(_dist_width_bins, 0.0F, 1.0F, _dist_w_title),
            axis::regular(_dist_height_bins, 0.0F, 1.0F, _dist_h_title));

    const float delta = 5.0F * std::numeric_limits<float>::epsilon();
    if (size && _size_histo_enabled) {
        const float s_min = min(size_stat) - delta;
        const float s_max = max(size_stat) + delta;
        _size_histo       = make_histogram(
            axis::regular{_size_bins, s_min, s_max, _size_title});
    }

    if (response && _response_histo_enabled) {
        const float r_min = min(response_stat) - delta;
        const float r_max = max(response_stat) + delta;
        _response_histo   = make_histogram(
            axis::regular{_response_bins, r_min, r_max, _response_title});
    }

    std::for_each(points.begin(), points.end(), [&](const cv::KeyPoint& kp) {
        if (distribution)
            _distribution(kp.pt.x / static_cast<float>(_img_width),
                          kp.pt.y / static_cast<float>(_img_height));
        if (response && _response_histo_enabled)
            _response_histo(kp.response);
        if (size && _size_histo_enabled)
            _size_histo(kp.size);
    });
}
}  // namespace sens_loc::analysis
