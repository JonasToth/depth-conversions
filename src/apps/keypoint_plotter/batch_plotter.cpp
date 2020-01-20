#include "batch_plotter.h"

#include "util/parallel_processing.h"

#include <fmt/core.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace sens_loc::apps {
bool batch_plotter::process_batch(int start, int end) const noexcept {
    return parallel_indexed_file_processing(
        start, end,
        [this](int idx) noexcept -> bool { return process_index(idx); });
}

bool batch_plotter::process_index(int idx) const noexcept {
    try {
        using cv::FileNode;
        using cv::FileStorage;
        using cv::read;

        const std::string input_file = fmt::format(_feature_file_pattern, idx);

        const FileStorage fs{input_file,
                             FileStorage::READ | FileStorage::FORMAT_YAML};

        // if 'keypoints'-key does not exist give an error.
        const FileNode            keypoints_node = fs["keypoints"];
        std::vector<cv::KeyPoint> keypoints;
        read(keypoints_node, keypoints);

        if (keypoints.empty())
            return false;

        std::string    original_image;
        const FileNode file_path_node = fs["source_path"];
        read(file_path_node, original_image, "");

        if (_target_image_file_pattern || original_image.empty()) {
            // If the feature file does not contain a path to file the features
            // were detected on (it should always) or if it was explicitly
            // overwritten, then this specified path must be used instead.
            if (_target_image_file_pattern)
                original_image = fmt::format(*_target_image_file_pattern, idx);
            // No image path that can be read as basis for plotting.
            else
                return false;
        }
        Ensures(!original_image.empty());

        const cv::Mat source_image =
            cv::imread(original_image, cv::IMREAD_UNCHANGED);
        if (source_image.empty())
            return false;

        cv::Mat converted_source;

        // Transform possible input images into the correct COLOR_BGR 8bit
        // color space.
        if (source_image.type() == CV_16UC1) {
            cv::Mat intermediate;
            source_image.convertTo(intermediate, CV_8UC1, 1. / 255.);
            cv::cvtColor(intermediate, converted_source, cv::COLOR_GRAY2BGR);
        } else if (source_image.type() == CV_8UC1) {
            cv::cvtColor(source_image, converted_source, cv::COLOR_GRAY2BGR);
        } else if (source_image.type() == CV_8UC3) {
            source_image.copyTo(converted_source);
        } else {
            return false;
        }

        cv::drawKeypoints(converted_source, keypoints, converted_source,
                          color_to_bgr::convert(_color),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS |
                              cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

        const std::string output_file = fmt::format(_ouput_file_pattern, idx);
        const bool write_success = cv::imwrite(output_file, converted_source);

        return write_success;
    } catch (const std::exception& e) {
        std::cerr << util::err{} << "Error occured while processing index "
                  << idx << "!\n"
                  << e.what() << "\n";
        return false;
    } catch (...) {
        std::cerr << util::err{}
                  << "Unspecified error occured while processing index " << idx
                  << "!\n";
        return false;
    }
}  // namespace sens_loc::apps
}  // namespace sens_loc::apps
