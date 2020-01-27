#ifndef DISTANCE_H_MUJPGVJL
#define DISTANCE_H_MUJPGVJL

#include <boost/histogram.hpp>
#include <gsl/gsl>

namespace sens_loc {

/// This namespace contains useful function for statistical analysis of
/// keypoints, descriptors and other relevant quantities.
namespace analysis {

/// Wrapper-struct to hold the common set of statistical values.
struct statistic {
    std::size_t count    = 0UL;
    float       min      = 0.0F;
    float       max      = 0.0F;
    float       median   = 0.0F;
    float       mean     = 0.0F;
    float       variance = 0.0F;
    float       stddev   = 0.0F;
    float       skewness = 0.0F;

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
    distance(gsl::span<const float> distances,
             unsigned int           bin_count = 50U,
             std::string            title     = "distance")
        : _data{distances}
        , _bin_count{bin_count}
        , _axis_title{std::move(title)} {
        analyze(_data);
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

  private:
    // Accumulators that are run over the dataset. This data stores the
    // results.
    gsl::span<const float> _data;

    unsigned int _bin_count  = 50UL;
    std::string  _axis_title = "distance";
    histo_t      _histo;
    statistic    _s;
};
}  // namespace analysis
}  // namespace sens_loc

#endif /* end of include guard: DISTANCE_H_MUJPGVJL */
