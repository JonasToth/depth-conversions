#include "min_dist.h"

#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <fmt/core.h>
#include <gsl/gsl>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <sens_loc/util/correctness_util.h>
#include <sens_loc/util/thread_analysis.h>
#include <stdexcept>
#include <string_view>
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

namespace {

template <cv::NormTypes NT>
struct element_access {
    using type                 = float;
    static constexpr int dtype = CV_32F;
};
template <>
struct element_access<cv::NormTypes::NORM_HAMMING> {
    using type                 = int;
    static constexpr int dtype = CV_32S;
};
template <>
struct element_access<cv::NormTypes::NORM_HAMMING2> {
    using type                 = int;
    static constexpr int dtype = CV_32S;
};

/// Calculate the minimal distance between descriptors within one image
/// and save this minimal distance.
/// This gives an overall idea of the spread of descriptors within an image.
template <cv::NormTypes NT>
class min_descriptor_distance {
  public:
    static const int data_type = element_access<NT>::dtype;
    using access_type          = typename element_access<NT>::type;

    min_descriptor_distance(gsl::not_null<std::mutex*>         m,
                            gsl::not_null<std::vector<float>*> global_data)
        : process_mutex{m}
        , global_min_distances{global_data} {}

    ~min_descriptor_distance()                                        = default;
    min_descriptor_distance(const min_descriptor_distance<NT>& other) = default;
    min_descriptor_distance(min_descriptor_distance<NT>&& other)      = default;

    min_descriptor_distance&
    operator=(const min_descriptor_distance<NT>& other) = default;
    min_descriptor_distance&
    operator=(min_descriptor_distance<NT>&& other) = default;

    void
    operator()(int /*idx*/,
               std::optional<std::vector<cv::KeyPoint>> keypoints,  // NOLINT
               std::optional<cv::Mat>                   descriptors) noexcept {
        Expects(!keypoints.has_value());
        Expects(descriptors.has_value());

        cv::Mat distances;
        cv::batchDistance(*descriptors, *descriptors, distances, data_type,
                          cv::noArray(),
                          /*normType=*/NT);

        // Calculate the minimal distances within that image.
        std::vector<float> local_min_distances;
        local_min_distances.reserve(distances.rows);
        for (decltype(distances.rows) row = 0; row < distances.rows; ++row) {
            // Make the distance to itself maximal, as that is known to
            // be zero and needs to be ignored.
            distances.at<access_type>(row, row) =
                std::numeric_limits<access_type>::max();

            cv::Mat r = distances.row(row);
            auto    d = gsl::narrow<float>(*std::min_element(
                r.begin<access_type>(), r.end<access_type>()));

            local_min_distances.push_back(d);
        }

        // Insert the findings into the global list of minimal distances.
        {
            std::lock_guard guard{*process_mutex};
            global_min_distances->insert(std::end(*global_min_distances),
                                         std::begin(local_min_distances),
                                         std::end(local_min_distances));
        }
    }

    /// Postprocess the findings of the minimal distances for each image to
    /// a coherent statistical finding. This will overwrite previous analysis
    /// and should only be called once.
    [[nodiscard]] auto postprocess(int bins) noexcept {
        // Lock the mutex, just in case. This method is not expected to be
        // run in parallel, but it could.
        std::lock_guard guard{*process_mutex};

        Expects(!global_min_distances->empty());

        auto [_, max_it] = std::minmax_element(
            std::begin(*global_min_distances), std::end(*global_min_distances));

        Expects(*_ >= 0.0F);

        // Histogramming
        using namespace boost::histogram;
        auto dist_histo = make_histogram(
            axis::regular<float, use_default, axis::option::none_t>(
                bins, 0.0F, *max_it + 0.01F));
        dist_histo.fill(*global_min_distances);

        using namespace boost::accumulators;
        accumulator_set<float, stats<tag::min, tag::max, tag::mean, tag::median,
                                     tag::variance(lazy), tag::skewness>>
            stat;

        std::for_each(std::begin(*global_min_distances),
                      std::end(*global_min_distances),
                      [&stat](float el) { stat(el); });


        return make_pair(dist_histo, stat);
    }

  private:
    // Data required for the parallel processing.
    gsl::not_null<std::mutex*>         process_mutex;
    gsl::not_null<std::vector<float>*> global_min_distances;
};

template <cv::NormTypes NT>
int analyze_min_distance_impl(std::string_view input_pattern,
                              int              start_idx,
                              int              end_idx) {
    using namespace sens_loc::apps;
    /// Guards min_distances in parallel access.
    std::mutex process_mutex;
    /// contains all minimal distances of each descriptor within an image.
    std::vector<float> global_min_distances;

    using visitor = statistic_visitor<min_descriptor_distance<NT>,
                                      required_data::descriptors>;

    auto f = parallel_visitation(start_idx, end_idx,
                                 visitor{input_pattern,
                                         gsl::not_null{&process_mutex},
                                         gsl::not_null{&global_min_distances}});

    auto [histogram, stat] = f.postprocess(25);

    std::cout << histogram << std::endl;
    using namespace boost::accumulators;
    std::cout << "==== Descriptor Distances\n"
              << "min:       " << min(stat) << "\n"
              << "max:       " << max(stat) << "\n"
              << "Mean:      " << mean(stat) << "\n"
              << "Median:    " << median(stat) << "\n"
              << "Variance:  " << variance(stat) << "\n"
              << "Skewness:  " << skewness(stat) << "\n";
    return 0;
}
}  // namespace

namespace sens_loc::apps {
int analyze_min_distance(std::string_view input_pattern,
                         int              start_idx,
                         int              end_idx,
                         cv::NormTypes    norm_to_use) {

#define SWITCH_CV_NORM(NORM_NAME)                                              \
    if (norm_to_use == cv::NormTypes::NORM_##NORM_NAME)                        \
        return analyze_min_distance_impl<cv::NormTypes::NORM_##NORM_NAME>(     \
            input_pattern, start_idx, end_idx);
    SWITCH_CV_NORM(L1)
    SWITCH_CV_NORM(L2)
    SWITCH_CV_NORM(L2SQR)
    SWITCH_CV_NORM(HAMMING2)
    SWITCH_CV_NORM(HAMMING)
#undef SWITCH_CV_NORM
    UNREACHABLE("unexpected norm type");  // LCOV_EXCL_LINE
}
}  // namespace sens_loc::apps
