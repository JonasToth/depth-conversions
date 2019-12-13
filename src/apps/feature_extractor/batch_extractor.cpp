#include "batch_extractor.h"

#include <chrono>
#include <fmt/core.h>
#include <opencv2/core.hpp>
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
                   cv::Feature2D&     detector) noexcept {
    // FIXME: Optimization oppurtinity to reuse the strings here
    const std::string                 f_name = fmt::format(in_pattern, idx);
    std::optional<math::image<uchar>> f      = load_file(f_name);

    if (!f)
        return false;

    auto [keypoints, descriptors] = compute_features(*f, detector);
    (void) descriptors;

    cv::Mat img_features;
    cv::drawKeypoints(f->data(), keypoints, img_features);

    const std::string out_name      = fmt::format(out_pattern, idx);
    const bool        write_success = cv::imwrite(out_name, img_features);
    return write_success;
}
}  // namespace

bool batch_extractor::process_batch(int start, int end) const noexcept {
    if (start > end)
        std::swap(start, end);

    bool batch_success = true;

    try {
        tf::Executor executor;
        tf::Taskflow tf;
        std::mutex   cout_mutex;
        int          fails = 0;

        tf.parallel_for(start, end + 1, 1,
                        [&cout_mutex, &batch_success, &fails, this](int idx) {
                            const bool success = process_index(
                                this->_input_pattern, this->_output_pattern,
                                idx, *this->_detector);
                            if (!success) {
                                std::lock_guard l(cout_mutex);
                                fails++;
                                std::cerr << util::err{};
                                std::cerr << "Could not process index \""
                                          << rang::style::bold << idx << "\""
                                          << rang::style::reset << "!\n";
                                batch_success = false;
                            }
                        });

        const auto before = std::chrono::steady_clock::now();
        executor.run(tf).wait();
        const auto after = std::chrono::steady_clock::now();

        Ensures(fails >= 0);

        std::cerr << util::info{};
        std::cerr << "Processing " << rang::style::bold
                  << std::abs(end - start) + 1 - fails << rang::style::reset
                  << " images took " << rang::style::bold
                  << std::chrono::duration_cast<std::chrono::seconds>(after -
                                                                      before)
                         .count()
                  << "" << rang::style::reset << " seconds!\n";

        if (fails > 0)
            std::cerr << util::warn{} << "Encountered " << rang::style::bold
                      << fails << rang::style::reset << " problematic files!\n";
    } catch (const std::exception& e) {
        std::cerr << util::err{} << "Problem occured during batch-processing!\n"
                  << "Message: " << e.what() << "\n";
        batch_success = false;
    } catch (...) {
        std::cerr << util::err{}
                  << "Fatal problem occured while batch-processing!\n"
                  << "Potential exhaustion of resources or an other system "
                     "problem!\n";
        batch_success = false;
    }

    return batch_success;
}

}  // namespace sens_loc::apps
