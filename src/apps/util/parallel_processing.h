#ifndef PARALLEL_PROCESSING_H_2FVRLMCH
#define PARALLEL_PROCESSING_H_2FVRLMCH

#include <chrono>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/util/console.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc::apps {

/// Helper function that processes a range of files based on index.
/// The boolean function \c f is applied to each function. Error handling
/// and reporting is done if \c f returns \c false.
///
/// \tparam BoolFunction Apply this functor for each index.
/// \param start,end inclusive range of integers for the files
/// \param f functor that is applied for each index
template <typename BoolFunction>
bool parallel_indexed_file_processing(int          start,
                                      int          end,
                                      BoolFunction f) noexcept {
    if (start > end)
        std::swap(start, end);

    bool batch_success = true;
    try {
        tf::Executor executor;
        tf::Taskflow tf;
        std::mutex   cout_mutex;
        int          fails = 0;

        tf.parallel_for(start, end + 1, 1,
                        [&cout_mutex, &batch_success, &fails, &f](int idx) {
                            const bool success = f(idx);
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

#endif /* end of include guard: PARALLEL_PROCESSING_H_2FVRLMCH */
