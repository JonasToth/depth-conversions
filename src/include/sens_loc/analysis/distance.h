#ifndef DISTANCE_H_MUJPGVJL
#define DISTANCE_H_MUJPGVJL

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <gsl/gsl>
#include <opencv2/core/persistence.hpp>

namespace sens_loc {

/// This namespace contains useful function for statistical analysis of
/// keypoints, descriptors and other relevant quantities.
namespace analysis {

/// Wrapper-struct to hold the common set of statistical values.
struct statistic {
    using accumulator_t = boost::accumulators::accumulator_set<
        float,
        boost::accumulators::stats<boost::accumulators::tag::count,
                                   boost::accumulators::tag::min,
                                   boost::accumulators::tag::max,
                                   boost::accumulators::tag::median,
                                   boost::accumulators::tag::mean,
                                   boost::accumulators::tag::variance(
                                       boost::accumulators::lazy),
                                   boost::accumulators::tag::skewness>>;

    std::size_t count    = 0UL;
    float       min      = 0.0F;
    float       max      = 0.0F;
    float       median   = 0.0F;
    float       mean     = 0.0F;
    float       variance = 0.0F;
    float       stddev   = 0.0F;
    float       skewness = 0.0F;

    static statistic make(const accumulator_t& accu) {
        statistic s;
        using namespace boost::accumulators;
        s.count    = boost::accumulators::count(accu);
        s.min      = boost::accumulators::min(accu);
        s.max      = boost::accumulators::max(accu);
        s.median   = boost::accumulators::median(accu);
        s.mean     = boost::accumulators::mean(accu);
        s.variance = boost::accumulators::variance(accu);
        s.stddev   = std::sqrt(s.variance);
        s.skewness = boost::accumulators::skewness(accu);
        return s;
    }

    void reset() noexcept {
        count    = 0UL;
        min      = 0.0F;
        max      = 0.0F;
        median   = 0.0F;
        mean     = 0.0F;
        variance = 0.0F;
        stddev   = 0.0F;
        skewness = 0.0F;
    }
};

/// Write-Functionality for OpenCVs-Filestorage API.
void write(cv::FileStorage& fs, const std::string& name, const statistic& stat);

/// This class processes 'float' datapoints and calculates both histograms
/// and basic statistical quantities, like the 'min', 'max', 'mean' and
/// others.
/// Its main use-case is to analyze distance information gathered while
/// processing image and matching data. It can be used for any scalar
/// quantity though.
class distance {
  public:
    using axis_t = boost::histogram::axis::regular<
        // 'float' values are tracked with the histogram.
        float,
        // do no transformation before insertion
        boost::histogram::axis::transform::id,
        // a string is the metadata for the axis (=title)
        std::string,
        // no overflow/underflow by construction.
        boost::histogram::axis::option::none_t>;
    using histo_t = decltype(boost::histogram::make_histogram(axis_t{}));


    distance() = default;
    explicit distance(gsl::span<const float> distances,
                      unsigned int           bin_count = 50U,
                      std::string            title     = "distance") noexcept
        : _bin_count{bin_count}
        , _axis_title{std::move(title)} {
        analyze(distances);
    }

    /// Analyses the provided distances. This will overwrite a previous
    /// analysis and dataset! Use this method to provide the data if the
    /// class was default constructed.
    void analyze(gsl::span<const float> distances, bool histo = true) noexcept;

    /// Return the reference to the potentially created histogram in \c analyze.
    /// If the histogram is not created, it will just be default constructed.
    /// \sa analyze
    [[nodiscard]] const histo_t& histogram() const noexcept { return _histo; }

    [[nodiscard]] std::size_t count() const noexcept { return _s.count; }
    [[nodiscard]] float       min() const noexcept { return _s.min; }
    [[nodiscard]] float       max() const noexcept { return _s.max; }
    [[nodiscard]] float       median() const noexcept { return _s.median; }
    [[nodiscard]] float       mean() const noexcept { return _s.mean; }
    [[nodiscard]] float       variance() const noexcept { return _s.variance; }
    [[nodiscard]] float       stddev() const noexcept { return _s.stddev; }
    [[nodiscard]] float       skewness() const noexcept { return _s.skewness; }

    /// Read-Only access to underlying statistical data.
    [[nodiscard]] const statistic& get_statistic() const noexcept { return _s; }

  private:
    unsigned int _bin_count  = 50UL;
    std::string  _axis_title = "distance";
    histo_t      _histo;
    statistic    _s;
};
}  // namespace analysis
}  // namespace sens_loc

#endif /* end of include guard: DISTANCE_H_MUJPGVJL */
