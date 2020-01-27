#ifndef DISTANCE_H_MUJPGVJL
#define DISTANCE_H_MUJPGVJL

#include <boost/histogram.hpp>
#include <gsl/gsl>
#include <memory>

namespace sens_loc {

/// This namespace contains useful function for statistical analysis of
/// keypoints, descriptors and other relevant quantities.
namespace analysis {

/// This class processes 'float' datapoints and calculates both histograms
/// and basic statistical quantities, like the 'min', 'max', 'mean' and
/// others.
/// Its main use-case is to analyze distance information gathered while
/// processing image and matching data. It can be used for any scalar
/// quantity though.
class distance {
  public:
    using histo_t = decltype(boost::histogram::make_histogram(
        boost::histogram::axis::regular<
            float,  // 'float' values are tracked with the histogram.
            boost::histogram::axis::transform::id,  // do no transformation
                                                    // before insertion
            std::string,  // a string is the metadata for the axis (=title)
            boost::histogram::axis::option::none_t>{}));  // no
                                                          // overflow/underflow
                                                          // by construction.

    distance() = default;
    distance(gsl::span<const float> distances,
             std::size_t            bin_count = 50UL,
             std::string            title     = "distance")
        : _data{distances}
        , _count{}
        , _min{}
        , _max{}
        , _median{}
        , _mean{}
        , _variance{}
        , _stddev{}
        , _skewness{}
        , _bin_count{bin_count}
        , _axis_title{std::move(title)}
        , _histo{} {
        analyze(_data);
    }

    /// Analyses the provided distances. This will overwrite a previous analysis
    /// and dataset! Use this method to provide the data if the class was
    /// default constructed.
    void analyze(gsl::span<const float> distances, bool histo = true) noexcept;

    [[nodiscard]] const histo_t& histogram() const noexcept { return _histo; }
    [[nodiscard]] std::size_t    count() const noexcept { return _count; }
    [[nodiscard]] float          min() const noexcept { return _min; }
    [[nodiscard]] float          max() const noexcept { return _max; }
    [[nodiscard]] float          median() const noexcept { return _median; }
    [[nodiscard]] float          mean() const noexcept { return _mean; }
    [[nodiscard]] float          variance() const noexcept { return _variance; }
    [[nodiscard]] float          stddev() const noexcept { return _stddev; }
    [[nodiscard]] float          skewness() const noexcept { return _skewness; }

  private:
    // Accumulators that are run over the dataset. This data stores the results.
    gsl::span<const float> _data;
    std::size_t            _count      = 0UL;
    float                  _min        = 0.0F;
    float                  _max        = 0.0F;
    float                  _median     = 0.0F;
    float                  _mean       = 0.0F;
    float                  _variance   = 0.0F;
    float                  _stddev     = 0.0F;
    float                  _skewness   = 0.0F;
    std::size_t            _bin_count  = 50UL;
    std::string            _axis_title = "distance";
    histo_t                _histo;
};
}  // namespace analysis
}  // namespace sens_loc

#endif /* end of include guard: DISTANCE_H_MUJPGVJL */
