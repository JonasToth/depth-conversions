#include "min_dist.h"

#include <algorithm>
#include <boost/histogram.hpp>
#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <fmt/core.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string_view>

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

template <cv::NormTypes NT>
int analyze_min_distance_impl(std::string_view input_pattern,
                              int              start_idx,
                              int              end_idx) {
    const int end_inclusive   = end_idx + 1;
    int       overall_success = 0;

    using T         = typename element_access<NT>::type;
    const int dtype = element_access<NT>::dtype;

    std::vector<float> min_distances;
    // Optimization, it is expected, that many minimal distances are inserted.
    // This will save multiple small allocations in the early growing stage.
    min_distances.reserve(1000);

    for (int i = start_idx; i < end_inclusive; ++i) {
        if (i % 50 == 0)
            std::cout << "Processing Img: " << i << "\n";

        try {
            using cv::FileNode;
            using cv::FileStorage;
            using cv::read;

            const std::string input_file = fmt::format(input_pattern, i);
            const FileStorage fs{input_file,
                                 FileStorage::READ | FileStorage::FORMAT_YAML};

            // if 'keypoints'-key does not exist give an error.
            const FileNode descriptors_node = fs["descriptors"];
            cv::Mat        descriptors;
            read(descriptors_node, descriptors);

            if (descriptors.empty())
                throw std::invalid_argument{"no keypoints in this file"};

            cv::Mat distances;
            cv::batchDistance(descriptors, descriptors, distances, dtype,
                              cv::noArray(), /*normType=*/NT);

            for (int row = 0; row < distances.rows; ++row) {
                // Make the distance to itself maximal, as that is known to
                // be zero and needs to be ignored.
                distances.at<T>(row, row) = std::numeric_limits<T>::max();

                cv::Mat r        = distances.row(row);
                auto    min_dist = gsl::narrow<float>(
                    *std::min_element(r.begin<T>(), r.end<T>()));

                min_distances.push_back(min_dist);
#if 0
                std::cout << "Minimal Distance for Descriptor   # " << row
                          << " = " << min_dist << "\n";
#endif
            }
        } catch (const std::exception& e) {
            std::cout << "Problemo!\n" << e.what() << "\n";
            overall_success = 1;
        } catch (...) {
            std::cout << "Problemo!\n";
            overall_success = 1;
        }
    }
    auto [min_it, max_it] =
        std::minmax_element(std::begin(min_distances), std::end(min_distances));

    std::cout << "Minimum distance: " << *min_it << "\n"
              << "Maximum distance: " << *max_it << "\n";


    std::cout << "Creating Histogram\n";
    // Histogramming
    using namespace boost::histogram;
    const int bin_count  = 50;
    auto      min_dist_h = make_histogram(
        axis::regular<float>(bin_count, 0.0, *max_it + (*max_it / bin_count)));
    min_dist_h.fill(min_distances);

    std::cout << min_dist_h << "\n";

    return overall_success;
}
}  // namespace

namespace sens_loc::apps {
int analyze_min_distance(std::string_view input_pattern,
                         int              start_idx,
                         int              end_idx,
                         cv::NormTypes    norm_to_use) {
    // NOLINT-NEXTLINE(cppcoreguidelines-macro-usage)
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
