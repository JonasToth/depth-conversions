#include "min_dist.h"

#include <algorithm>
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

    for (int i = start_idx; i < end_inclusive; ++i) {
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

            std::cout << "Columns: " << descriptors.cols << "\n";
            cv::Mat distances;
            cv::batchDistance(descriptors, descriptors, distances, dtype,
                              cv::noArray(), /*normType=*/NT);

            for (int row = 0; row < distances.rows; ++row) {
                // Make the distance to itself maximal, as that is known to
                // be zero and needs to be ignored.
                distances.at<T>(row, row) = std::numeric_limits<T>::max();

                cv::Mat r        = distances.row(row);
                float   min_dist = *std::min_element(r.begin<T>(), r.end<T>());
                std::cout << "Minimal Distance for Descriptor   # " << row
                          << " = " << min_dist << "\n";

                if (min_dist == 0.) {
                    for (int c = 0; c < distances.cols; ++c) {
                        std::cout << std::setw(4)
                                  << (int) distances.at<T>(row, c);
                    }
                    std::cout << "\n";
                }
            }
        } catch (const std::exception& e) {
            std::cout << "Problemo!\n" << e.what() << "\n";
            overall_success = 1;
        } catch (...) {
            std::cout << "Problemo!\n";
            overall_success = 1;
        }
    }
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
