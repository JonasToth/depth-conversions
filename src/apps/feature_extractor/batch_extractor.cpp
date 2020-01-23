#include "batch_extractor.h"

#include "util/parallel_processing.h"

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

/// Computes both keypoints and corresponding descriptors and returns them
/// as pair.
std::pair<std::vector<cv::KeyPoint>, cv::Mat>
compute_features(const math::image<uchar>& img,
                 cv::Feature2D&            detector,
                 cv::Feature2D&            descriptor) noexcept {

    // The image itself is the mask for feature detection.
    // That is the reason, because the pixels with 0 as value do not contain
    // any information on the geometry.
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(img.data(), keypoints, img.data());

    cv::Mat descriptors;
    descriptor.compute(img.data(), keypoints, descriptors);

    return std::make_pair(std::move(keypoints), std::move(descriptors));
}

/// Applies \c detector to each image loaded from \c in_pattern, substituted
/// with \c idx.
/// The keypoints and descriptors are detected and written in a YAML-file
/// in \c out_pattern, substituted with \c idx.
bool process_detector(const math::image<uchar>& image,
                      cv::Feature2D&            detector,
                      cv::Feature2D&            descriptor,
                      const std::string&        out_file,
                      const std::string&        in_file) noexcept {

    const auto [keypoints, descriptors] =
        compute_features(image, detector, descriptor);

    try {
        using cv::FileNode;
        using cv::FileStorage;
        using cv::write;

        FileStorage fs{out_file, FileStorage::WRITE | FileStorage::FORMAT_YAML};

        detector.write(fs);
        write(fs, "source_path", in_file);
        write(fs, "keypoints", keypoints);
        write(fs, "descriptors", descriptors);

        return true;
    } catch (...) {
        return false;
    }
}
}  // namespace

/// Runs all \c detectors this file identified by \c idx and \c in_pattern.
bool batch_extractor::process_index(int idx) const noexcept {
    const std::string                 p = fmt::format(_input_pattern, idx);
    std::optional<math::image<uchar>> f = load_file(p);

    if (!f)
        return false;

    return process_detector(*f, *_detector, *_descriptor,
                            fmt::format(_ouput_pattern, idx), p);
}

bool batch_extractor::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(
        start, end, [this](int idx) noexcept { return process_index(idx); });
}
}  // namespace sens_loc::apps
