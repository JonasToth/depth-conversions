#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cmath>
#include <limits>
#include <sens_loc/analysis/distance.h>

namespace sens_loc::analysis {

void distance::analyze(gsl::span<const float> distances, bool histo) noexcept {
    _histo.reset();
    _s.reset();

    Expects(!distances.empty());
    _data    = distances;
    _s.count = _data.size();

    using namespace boost::accumulators;
    accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean,
                                 tag::variance(lazy), tag::skewness>>
        stat;
    std::for_each(std::begin(_data), std::end(_data),
                  [&](auto e) noexcept -> void { stat(e); });

    // If the namepsace is not provided, the call is ambigous.
    _s.min      = boost::accumulators::min(stat);
    _s.max      = boost::accumulators::max(stat);
    _s.median   = boost::accumulators::median(stat);
    _s.mean     = boost::accumulators::mean(stat);
    _s.variance = boost::accumulators::variance(stat);
    _s.stddev   = std::sqrt(_s.variance);
    _s.skewness = boost::accumulators::skewness(stat);

    if (!histo)
        return;

    using namespace boost::histogram;
    const float h_min = _s.min - 5.F * std::numeric_limits<float>::epsilon();
    const float h_max = _s.max + 5.F * std::numeric_limits<float>::epsilon();

    _histo = make_histogram(
        axis::regular<float>(_bin_count, h_min, h_max, _axis_title));
    _histo.fill(_data);
}

}  // namespace sens_loc::analysis
