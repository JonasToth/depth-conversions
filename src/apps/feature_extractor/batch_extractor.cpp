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
std::optional<math::image<uchar>> load_file(std::string_view name) noexcept {
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(name.data(), cv::IMREAD_UNCHANGED);

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

/// This overload allows having different algorithms for keypoint extraction
/// and descriptor calculation. For example FREAK does not have a descriptor
/// and is required to use a different descriptor.
std::pair<std::vector<cv::KeyPoint>, cv::Mat>
compute_features(const math::image<uchar>& img,
                 cv::Feature2D&            detector,
                 cv::Feature2D&            descriptor) noexcept {
    // The image itself is the mask for feature detection.
    // That is the reason, because the pixels with 0 as value do not contain
    // any information on the geometry.
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(img.data(), keypoints, img.data());

    // Compute the descriptors at the keypoints.
    cv::Mat descriptors;
    descriptor.compute(img.data(), keypoints, descriptors);

    return std::make_pair(std::move(keypoints), std::move(descriptors));
}

/// Applies \c detector to each image loaded from \c in_pattern, substituted
/// with \c idx.
/// The keypoints and descriptors are detected and written in a YAML-file
/// in \c out_pattern, substituted with \c idx.
bool process_detector(const math::image<uchar>& image,
                      const Detector&           detector,
                      std::string_view          out_file,
                      std::string_view          in_file) noexcept {
    const auto [keypoints, descriptors] = [&]() {
        if (detector.descriptor.empty())
            return compute_features(image, *detector.detector);
        return compute_features(image, *detector.detector,
                                *detector.descriptor);
    }();

    try {
        using cv::FileNode;
        using cv::FileStorage;
        using cv::write;

        FileStorage fs{out_file.data(),
                       FileStorage::WRITE | FileStorage::FORMAT_YAML};

        detector.detector->write(fs);
        if (!detector.descriptor.empty())
            detector.descriptor->write(fs);

        write(fs, "source_path", in_file.data());
        write(fs, "keypoints", keypoints);
        write(fs, "descriptors", descriptors);

        return true;
    } catch (...) {
        return false;
    }
}

/// Runs all \c detectors this file identified by \c idx and \c in_pattern.
bool process_index(int                          idx,
                   std::string_view             in_pattern,
                   const std::vector<Detector>& detectors) noexcept {
    const std::string                 f_name = fmt::format(in_pattern, idx);
    std::optional<math::image<uchar>> f      = load_file(f_name);
    if (!f)
        return false;

    bool success = true;
    for (const auto& detector : detectors) {
        const std::string out_name = fmt::format(detector.output_pattern, idx);
        success &= process_detector(*f, detector, out_name, f_name);
    }
    return success;
}

}  // namespace

bool batch_extractor::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(
        start, end, [this](int idx) noexcept -> bool {
            return process_index(idx, this->_input_pattern, this->_detectors);
        });
}
}  // namespace sens_loc::apps
