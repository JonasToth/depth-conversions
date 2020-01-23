#ifndef MINIMAL_DESCRIPTOR_DISTANCE_H_EMR0VAQO
#define MINIMAL_DESCRIPTOR_DISTANCE_H_EMR0VAQO

#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <gsl/gsl>
#include <iostream>
#include <limits>
#include <mutex>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <sens_loc/util/thread_analysis.h>
#include <type_traits>

namespace sens_loc::apps {

namespace detail {
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
}  // namespace detail

/// Calculate the minimal distance between descriptors within one image
/// and save this minimal distance.
/// This gives an overall idea of the spread of descriptors within an image.
template <cv::NormTypes NT>
class min_descriptor_distance {
  public:
    static const int data_type = detail::element_access<NT>::dtype;
    using access_type          = typename detail::element_access<NT>::type;

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

}  // namespace sens_loc::apps

#endif /* end of include guard: MINIMAL_DESCRIPTOR_DISTANCE_H_EMR0VAQO */
