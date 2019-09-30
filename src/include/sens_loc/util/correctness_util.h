#ifndef CORRECTNESS_UTIL_H_BJPG9UMQ
#define CORRECTNESS_UTIL_H_BJPG9UMQ

#include <exception>
#include <gsl/gsl>
#include <iostream>
#include <opencv2/core.hpp>
#include <sens_loc/util/console.h>
#include <string_view>

namespace sens_loc { namespace util {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define UNREACHABLE(msg) ::sens_loc::util::unreachable(msg, __FILE__, __LINE__)

/// 'unreachable' is inspired by 'llvm_unreachable'.
[[noreturn]] inline void unreachable(const std::string_view message = {},
                                     const std::string_view file    = {},
                                     int                    line    = -1) {
    std::cerr << err{} << file << ":" << line
              << ": Program ran into an unreachable!\n"
              << info{} << message << "\n"
              << "Terminating!\n";
    std::exit(1);
}

/// Calculate the average pixel error of the two images.
/// Expects that both images are of same type and dimension.
/// This function uses the L1-Norm as error measure and divides it by the
/// number of pixels.
inline double average_pixel_error(const cv::Mat &i1,
                                  const cv::Mat &i2) noexcept {
    Expects(i1.cols == i2.cols);
    Expects(i1.rows == i2.rows);
    Expects(i1.type() == i2.type());
    Expects(i1.channels() == i2.channels());

    const size_t n_pixels = i1.cols * i1.rows * i1.channels();
    const double result   = cv::norm(i1, i2, cv::NORM_L2);

    Ensures(result >= 0.);
    return result / double(n_pixels);
}
}}  // namespace sens_loc::util

#endif /* end of include guard: CORRECTNESS_UTIL_H_BJPG9UMQ */
