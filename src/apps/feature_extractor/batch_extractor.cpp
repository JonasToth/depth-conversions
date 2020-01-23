#include "batch_extractor.h"

#include "util/parallel_processing.h"

#include <algorithm>
#include <chrono>
#include <fmt/core.h>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/features2d.hpp>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>
#include <sens_loc/util/console.h>
#include <taskflow/taskflow.hpp>
#include <utility>

namespace sens_loc::apps {

namespace {
/// Expects to load a 16bit grayscale image.
/// It converts those images to 8bit grayscale images that will be processed.
std::optional<math::image<uchar>> load_file(const std::string& name) noexcept {
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(name, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return {};

    return math::convert<uchar>(
        math::image<ushort>(depth_image->data() / 255.));  // NOLINT
}
}  // namespace

bool batch_extractor::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(
        start, end, [this](int idx) noexcept { return process_index(idx); });
}

bool batch_extractor::process_index(int idx) const noexcept {
    const std::string                 p = fmt::format(_input_pattern, idx);
    std::optional<math::image<uchar>> f = load_file(p);

    if (!f)
        return false;

    return process_detector(*f, fmt::format(_ouput_pattern, idx), p);
}

/// Applies \c detector to each image loaded from \c in_pattern, substituted
/// with \c idx.
/// The keypoints and descriptors are detected and written in a YAML-file
/// in \c out_pattern, substituted with \c idx.
bool batch_extractor::process_detector(const math::image<uchar>& image,
                                       const std::string&        out_file,
                                       const std::string&        in_file) const
    noexcept {

    const auto [keypoints, descriptors] = compute_features(image);

    try {
        using cv::FileNode;
        using cv::FileStorage;
        using cv::write;

        FileStorage fs{out_file, FileStorage::WRITE | FileStorage::FORMAT_YAML};
        write(fs, "source_path", in_file);
        write(fs, "keypoints", keypoints);
        write(fs, "descriptors", descriptors);

        return true;
    } catch (...) {
        return false;
    }
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat>
batch_extractor::compute_features(const math::image<uchar>& img) const
    noexcept {
    using namespace std;

    // The image itself is the mask for feature detection.
    // That is the reason, because the pixels with 0 as value do not contain
    // any information on the geometry.
    vector<cv::KeyPoint> keypoints;
    _detector->detect(img.data(), keypoints, img.data());

    // Removes every keypoint that is matched by the '_keypoint_filter'.
    if (_keypoint_filter) {
        auto new_end =
            remove_if(begin(keypoints), end(keypoints), *_keypoint_filter);
        keypoints.erase(new_end, end(keypoints));
    }

    cv::Mat descriptors;
    _descriptor->compute(img.data(), keypoints, descriptors);

    return make_pair(move(keypoints), move(descriptors));
}
}  // namespace sens_loc::apps
