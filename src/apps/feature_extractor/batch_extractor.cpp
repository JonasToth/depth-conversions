#include "batch_extractor.h"

#include "util/parallel_processing.h"

#include <chrono>
#include <fmt/core.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>
#include <sens_loc/util/console.h>
#include <taskflow/taskflow.hpp>
#include <utility>

namespace sens_loc::apps {

namespace {
std::optional<math::image<uchar>> load_file(const std::string& name) noexcept {
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(name, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return {};

    return math::convert<uchar>(
        math::image<ushort>(depth_image->data() / 255.));  // NOLINT
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat>
compute_features(const math::image<uchar>& img,
                 cv::Feature2D&            detector) noexcept {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;

    // The image itself is the mask for feature detection.
    // That is the reason, because the pixels with 0 as value do not contain
    // any information on the geometry.
    detector.detectAndCompute(img.data(), img.data(), keypoints, descriptors);

    return std::make_pair(std::move(keypoints), std::move(descriptors));
}

bool process_index(const std::string& in_pattern,
                   const std::string& out_pattern,
                   int                idx,
                   cv::Feature2D&     detector,
                   feature_color      color) noexcept {
    // FIXME: Optimization oppurtinity to reuse the strings here
    const std::string                 f_name = fmt::format(in_pattern, idx);
    std::optional<math::image<uchar>> f      = load_file(f_name);

    if (!f)
        return false;

    auto [keypoints, descriptors] = compute_features(*f, detector);
    (void) descriptors;

    cv::Mat img_features;
    cv::drawKeypoints(f->data(), keypoints, img_features,
                      color_to_rgb::convert(color),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    const std::string out_name      = fmt::format(out_pattern, idx);
    const bool        write_success = cv::imwrite(out_name, img_features);
    return write_success;
}
}  // namespace

bool batch_extractor::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(start, end, [this](int idx) {
        return process_index(this->_input_pattern,
                             this->_detector.output_pattern, idx,
                             *this->_detector.detector, this->_detector.color);
    });
}
}  // namespace sens_loc::apps
