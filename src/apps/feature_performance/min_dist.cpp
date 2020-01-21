#include "min_dist.h"

#include "minimal_descriptor_distance.h"

#include <algorithm>
#include <boost/histogram/ostream.hpp>
#include <cstdint>
#include <fmt/core.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string_view>
#include <util/batch_visitor.h>
#include <util/statistic_visitor.h>

namespace {
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
    auto f        = parallel_visitation(
        start_idx, end_idx,
        visitor{input_pattern, &process_mutex, &global_min_distances});
    auto histogram = f.postprocess(50);

    std::cout << histogram << std::endl;

    return 0;
}
}  // namespace

namespace sens_loc::apps {
int analyze_min_distance(std::string_view input_pattern,
                         int              start_idx,
                         int              end_idx,
                         cv::NormTypes    norm_to_use) {
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
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
