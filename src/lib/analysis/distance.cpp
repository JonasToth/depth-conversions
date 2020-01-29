#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/util/console.h>
#include <stdexcept>

namespace sens_loc::analysis {

void distance::analyze(gsl::span<const float> distances, bool histo) noexcept {
    _histo.reset();
    _s.reset();

    if (distances.empty())
        return;

    _s.count = distances.size();

    statistic::accumulator_t stat;
    try {
        std::for_each(std::begin(distances), std::end(distances),
                      [&](auto e) { stat(e); });
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create statistics for distance.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }

    try {
        // If the namespace is not provided, the call is ambigous.
        _s = statistic::make(stat);
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{} << "Can not extract accumulator.\n"
                  << "Message: " << e.what() << "\n";
    }

    if (!histo)
        return;

    const float h_min = _s.min - 5.F * std::numeric_limits<float>::epsilon();
    const float h_max = _s.max + 5.F * std::numeric_limits<float>::epsilon();

    try {
        _histo = boost::histogram::make_histogram(
            axis_t(_bin_count, h_min, h_max, _axis_title));
        _histo.fill(distances);
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create histogram for distance.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }
}

}  // namespace sens_loc::analysis
