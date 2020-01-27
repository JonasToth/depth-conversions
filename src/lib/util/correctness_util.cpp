#include <exception>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>


namespace sens_loc::util {

[[noreturn]] void unreachable(const std::string_view message,
                              const std::string_view file,
                              const int              line) noexcept {
    std::cerr << err{} << file << ":" << line
              << ": Program ran into an unreachable!\n"
              << info{} << message << "\n"
              << "Terminating!\n";
    std::exit(1);
}

double average_pixel_error(const cv::Mat& i1, const cv::Mat& i2) noexcept {
    Expects(i1.cols == i2.cols);
    Expects(i1.rows == i2.rows);
    Expects(i1.type() == i2.type());
    Expects(i1.channels() == i2.channels());

    const size_t n_pixels = i1.cols * i1.rows * i1.channels();
    const double result   = cv::norm(i1, i2, cv::NORM_L2);

    Ensures(result >= 0.);
    return result / double(n_pixels);
}

}  // namespace sens_loc::util
