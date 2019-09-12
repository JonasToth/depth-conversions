#include "batch_converter.h"

#include <chrono>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/util/console.h>
#include <sens_loc/io/image.h>
#include <fmt/core.h>
#include <taskflow/taskflow.hpp>
#include <thread>

namespace sens_loc { namespace apps {
bool batch_converter::process_index(int idx) const noexcept {
    Expects(!_files.input.empty());

    const std::string      input_file = fmt::format(_files.input, idx);
    std::optional<cv::Mat> depth_image =
        io::load_image(input_file, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return false;
    Expects((*depth_image).type() == CV_16U);

    return this->process_file(*depth_image, idx);
}

bool batch_converter::process_batch(int start, int end) const noexcept {
    Expects(start - end != 0);

    int fails       = 0;
    int return_code = 0;
    {
        tf::Executor executor;
        tf::Taskflow tf;
        std::mutex   cout_mutex;

        tf.parallel_for(start, end + 1, start < end ? 1 : -1,
                        [&cout_mutex, &return_code, &fails, this](int idx) {
                            const bool success = this->process_index(idx);
                            if (!success) {
                                std::lock_guard l(cout_mutex);
                                fails++;
                                std::cerr << util::err{};
                                std::cerr << "Could not process index \""
                                          << rang::style::bold << idx << "\""
                                          << rang::style::reset << "!\n";
                                return_code = 1;
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
    }

    return return_code;
}

}}  // namespace sens_loc::apps
