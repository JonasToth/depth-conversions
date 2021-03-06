#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS
#include "min_dist.h"

#include <algorithm>
#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <fmt/core.h>
#include <fstream>
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
#include <sens_loc/analysis/distance.h>
#include <sens_loc/io/histogram.h>
#include <sens_loc/util/correctness_util.h>
#include <sens_loc/util/thread_analysis.h>
#include <stdexcept>
#include <string_view>
#include <util/batch_visitor.h>
#include <util/common_structures.h>
#include <util/statistic_visitor.h>

using namespace std;

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

struct distance_stat_data {
    distance_stat_data() = default;

    void insert_distances(gsl::span<float> distances) noexcept {
        lock_guard l{_process_mutex};
        _global_min_distances.insert(end(_global_min_distances),
                                     begin(distances), end(distances));
    }

    vector<float> extract() noexcept {
        lock_guard    l{_process_mutex};
        vector<float> r       = move(_global_min_distances);
        _global_min_distances = vector<float>();
        return r;
    };

  private:
    mutex                               _process_mutex;
    vector<float> _global_min_distances GUARDED_BY(_process_mutex);
};

/// Calculate the minimal distance between descriptors within one image
/// and save this minimal distance.
/// This gives an overall idea of the spread of descriptors within an image.
template <cv::NormTypes NT>
class min_descriptor_distance {
  public:
    static const int data_type = element_access<NT>::dtype;
    using access_type          = typename element_access<NT>::type;

    min_descriptor_distance(distance_stat_data& data)
        : accumulated_data{data} {}

    void operator()(int /*idx*/,
                    optional<vector<cv::KeyPoint>> keypoints,  // NOLINT
                    optional<cv::Mat>              descriptors) noexcept {
        Expects(!keypoints.has_value());
        Expects(descriptors.has_value());

        if (descriptors->rows == 0)
            return;

        cv::Mat distances;
        cv::batchDistance(*descriptors, *descriptors, distances, data_type,
                          cv::noArray(),
                          /*normType=*/NT);

        // Calculate the minimal distances within that image.
        vector<float> local_min_distances;
        local_min_distances.reserve(distances.rows);
        for (decltype(distances.rows) row = 0; row < distances.rows; ++row) {
            // Make the distance to itself maximal, as that is known to
            // be zero and needs to be ignored.
            distances.at<access_type>(row, row) =
                numeric_limits<access_type>::max();

            cv::Mat r = distances.row(row);
            auto    d = gsl::narrow<float>(
                *min_element(r.begin<access_type>(), r.end<access_type>()));

            local_min_distances.push_back(d);
        }

        accumulated_data.insert_distances(local_min_distances);
    }

    /// Postprocess the findings of the minimal distances for each image to
    /// a coherent statistical finding. This will overwrite previous analysis
    /// and should only be called once.
    size_t postprocess(const optional<string>& stat_file,
                       const optional<string>& min_dist_histo) noexcept {
        // Lock the mutex, just in case. This method is not expected to be
        // run in parallel, but it could.
        vector<float> global_distances = accumulated_data.extract();

        sort(begin(global_distances), end(global_distances));
        const auto                   bins = 25UL;
        sens_loc::analysis::distance distance_stat{
            global_distances, bins, "Minimal Intra Image Descriptor Distances"};

        if (stat_file) {
            cv::FileStorage stat_out{*stat_file,
                                     cv::FileStorage::WRITE |
                                         cv::FileStorage::FORMAT_YAML};
            stat_out.writeComment("This file contains the statistical data for "
                                  "the distance to the closest descriptor.");
            write(stat_out, "descriptor_distance",
                  distance_stat.get_statistic());
            stat_out.release();
        } else {
            cout << "==== Descriptor Distances\n"
                 << "min:       " << distance_stat.min() << "\n"
                 << "max:       " << distance_stat.max() << "\n"
                 << "Mean:      " << distance_stat.mean() << "\n"
                 << "Median:    " << distance_stat.median() << "\n"
                 << "Variance:  " << distance_stat.variance() << "\n"
                 << "StdDev:    " << distance_stat.stddev() << "\n"
                 << "Skewness:  " << distance_stat.skewness() << "\n";
        }
        if (min_dist_histo) {
            ofstream gnuplot_data{*min_dist_histo};
            gnuplot_data << sens_loc::io::to_gnuplot(distance_stat.histogram())
                         << endl;
        } else {
            cout << distance_stat.histogram() << "\n";
        }
        return global_distances.size();
    }

  private:
    // Data required for the parallel processing.
    distance_stat_data& accumulated_data;
};

template <cv::NormTypes NT>
int analyze_min_distance_impl(sens_loc::util::processing_input in,
                              const optional<string>&          stat_file,
                              const optional<string>&          min_dist_histo) {
    using namespace sens_loc::apps;

    using visitor = statistic_visitor<min_descriptor_distance<NT>,
                                      required_data::descriptors>;

    distance_stat_data data;
    auto               f =
        parallel_visitation(in.start, in.end, visitor{in.input_pattern, data});

    size_t n_elements = f.postprocess(stat_file, min_dist_histo);

    return n_elements > 0UL ? 0 : 1;
}
}  // namespace

namespace sens_loc::apps {
int analyze_min_distance(util::processing_input  in,
                         cv::NormTypes           norm_to_use,
                         const optional<string>& stat_file,
                         const optional<string>& min_dist_histo) {

#define SWITCH_CV_NORM(NORM_NAME)                                              \
    if (norm_to_use == cv::NormTypes::NORM_##NORM_NAME)                        \
        return analyze_min_distance_impl<cv::NormTypes::NORM_##NORM_NAME>(     \
            in, stat_file, min_dist_histo);
    SWITCH_CV_NORM(L1)
    SWITCH_CV_NORM(L2)
    SWITCH_CV_NORM(L2SQR)
    SWITCH_CV_NORM(HAMMING2)
    SWITCH_CV_NORM(HAMMING)
#undef SWITCH_CV_NORM
    UNREACHABLE("unexpected norm type");  // LCOV_EXCL_LINE
}
}  // namespace sens_loc::apps
