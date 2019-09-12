#ifndef BATCH_CONVERTER_H_XDIRBPHG
#define BATCH_CONVERTER_H_XDIRBPHG

#include <chrono>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/util/console.h>
#include <string>
#include <taskflow/taskflow.hpp>
#include <thread>

/// Just local helper for batch conversion tasks over a given index range.

namespace sens_loc { namespace apps {

struct file_patterns {
    std::string input;
    std::string output;
    std::string horizontal;
    std::string vertical;
    std::string diagonal;
    std::string antidiagonal;
};

void check_output_exists(const file_patterns &files) {
    if (files.output.empty()) {
        std::cerr << util::err{};
        std::cerr << "output pattern required!\n";
        throw std::invalid_argument{"output pattern required\n"};
    }
}


class batch_converter {
  public:
    batch_converter(const file_patterns &files)
        : _files{files} {}

    /// Process the whole batch calling 'process_file' for each index.
    /// Returns 'false' if any of the indices fails.
    bool process_batch(int start, int end) const noexcept;

    virtual ~batch_converter() = default;

  protected:
    const file_patterns &_files;

  private:
    /// Method to process exactly on file. This method is expected to have
    /// no sideeffects and is called in parallel.
    virtual bool process_file(int idx) const noexcept = 0;
};

inline bool batch_converter::process_batch(int start, int end) const noexcept {
    Expects(start - end != 0);

    int fails       = 0;
    int return_code = 0;
    {
        tf::Executor executor;
        tf::Taskflow tf;
        std::mutex   cout_mutex;

        tf.parallel_for(start, end + 1, start < end ? 1 : -1,
                        [&cout_mutex, &return_code, &fails, this](int idx) {
                            const bool success = this->process_file(idx);
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

#endif /* end of include guard: BATCH_CONVERTER_H_XDIRBPHG */
