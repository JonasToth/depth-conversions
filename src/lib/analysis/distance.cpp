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

void distance::analyze(gsl::span<const float> distances) noexcept {
    Expects(!distances.empty());
    _data  = distances;
    _count = _data.size();

    using namespace boost::accumulators;
    accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean,
                                 tag::variance(lazy), tag::skewness>>
        stat;
    std::for_each(std::begin(_data), std::end(_data),
                  [&](auto e) noexcept -> void { stat(e); });

    _min      = boost::accumulators::min(stat);
    _max      = boost::accumulators::max(stat);
    _median   = boost::accumulators::median(stat);
    _mean     = boost::accumulators::mean(stat);
    _variance = boost::accumulators::variance(stat);
    _stddev   = std::sqrt(_variance);
    _skewness = boost::accumulators::skewness(stat);

    using namespace boost::histogram;
    float histo_min = _min - 5.F * std::numeric_limits<float>::epsilon();
    float histo_max = _max + 5.F * std::numeric_limits<float>::epsilon();
    _histo =
        make_histogram(axis::regular<float>(_bin_count, histo_min, histo_max));
    _histo.fill(_data);
}

}  // namespace sens_loc::analysis
