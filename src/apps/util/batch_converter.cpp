#include "batch_converter.h"

#include <chrono>
#include <fmt/core.h>
#include <gsl/gsl>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <taskflow/taskflow.hpp>
#include <thread>

namespace sens_loc::apps {
bool batch_converter::process_index(int idx) const noexcept {
    Expects(!_files.input.empty());

    const std::string input_file = fmt::format(_files.input, idx);
    std::optional<math::image<ushort>> depth_image =
        io::load_image<ushort>(input_file, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return false;

    std::optional<math::image<double>> pp_image =
        this->preprocess_depth(*depth_image);

    if (!pp_image)
        return false;

    return this->process_file(*pp_image, idx);
}

std::optional<math::image<double>>
batch_converter::preprocess_depth(const math::image<ushort>& depth_image) const
    noexcept {
    return math::convert<double>(depth_image);
}

bool batch_converter::process_batch(int start, int end) const noexcept {
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
                            const bool success = this->process_index(idx);
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
