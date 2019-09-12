#ifndef CORRECTNESS_UTIL_H_BJPG9UMQ
#define CORRECTNESS_UTIL_H_BJPG9UMQ

#include <exception>
#include <iostream>
#include <sens_loc/util/console.h>
#include <string_view>

namespace sens_loc { namespace util {

/// 'unreachable' is inspired by 'llvm_unreachable'.
[[noreturn]] inline void unreachable(const std::string_view message = {},
                                     const std::string_view file    = {},
                                     int                    line    = -1) {
    std::cerr << err{} << file << ":" << line
              << ": Program ran into an unreachable!\n"
              << info{} << message << "\n"
              << "Terminating!\n";
    std::terminate();
}
}}  // namespace sens_loc::util

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define UNREACHABLE(msg) ::sens_loc::util::unreachable(msg, __FILE__, __LINE__)

#endif /* end of include guard: CORRECTNESS_UTIL_H_BJPG9UMQ */
