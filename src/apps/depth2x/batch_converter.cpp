#include "batch_converter.h"

#include <chrono>
#include <fmt/core.h>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <taskflow/taskflow.hpp>
#include <thread>

namespace sens_loc::apps {
bool batch_converter::process_index(int idx) const noexcept {
    Expects(!_files.input.empty());

    const std::string      input_file = fmt::format(_files.input, idx);
    std::optional<cv::Mat> depth_image =
        io::load_image(input_file, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return false;
    if ((*depth_image).type() != CV_16U)
        return false;

    cv::Mat pp_image = this->preprocess_depth(std::move(*depth_image));
    return this->process_file(std::move(pp_image), idx);
}

bool batch_converter::process_batch(int start, int end) const noexcept {
    Expects(start - end != 0);

    int  fails         = 0;
    bool batch_success = true;
    try {
        tf::Executor executor;
        tf::Taskflow tf;
        std::mutex   cout_mutex;

        tf.parallel_for(start, end + 1, start < end ? 1 : -1,
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
    } catch (const std::exception &e) {
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
}  // namespace sens_loc::apps

cv::Mat batch_pinhole_converter::preprocess_depth(cv::Mat depth_image) const
    noexcept {
    switch (_input_depth_type) {
    case depth_type::orthografic:
        return conversion::depth_to_laserscan<double, ushort>(depth_image,
                                                              intrinsic);
    case depth_type::euclidean:
        cv::Mat depth_double(depth_image.rows, depth_image.cols, CV_64F);
        depth_image.convertTo(depth_double, CV_64F);
        return depth_double;
    }
    UNREACHABLE("Switch is exhaustive");
}
}  // namespace sens_loc::apps
