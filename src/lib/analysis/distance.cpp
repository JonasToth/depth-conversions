#include <cmath>
#include <iostream>
#include <limits>
#include <opencv2/core/persistence.hpp>
#include <sens_loc/analysis/distance.h>
#include <sens_loc/math/rounding.h>
#include <sens_loc/util/console.h>
#include <stdexcept>

namespace sens_loc::analysis {

void write(cv::FileStorage&   fs,
           const std::string& name,
           const statistic&   stat) {
    fs << name << "{";
    fs << "count" << gsl::narrow<int>(stat.count);
    fs << "min" << math::roundn(stat.min, 4);
    fs << "max" << math::roundn(stat.max, 4);
    fs << "median" << math::roundn(stat.median, 4);
    fs << "mean" << math::roundn(stat.mean, 4);
    fs << "variance" << math::roundn(stat.variance, 4);
    fs << "stddev" << math::roundn(stat.stddev, 4);
    fs << "skewness" << math::roundn(stat.skewness, 4);
    fs << "decentils"
       << "[";
    for (float decentil : stat.decentils)
        fs << decentil;
    fs << "]";

    fs << "}";
}

void distance::analyze(gsl::span<const float> distances, bool histo) noexcept {
    Expects(std::is_sorted(std::begin(distances), std::end(distances)));
    _histo.reset();
    _s.reset();

    if (distances.empty())
        return;

    _s.count = distances.size();

    try {
        // If the namespace is not provided, the call is ambigous.
        _s = statistic::make(distances);
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
