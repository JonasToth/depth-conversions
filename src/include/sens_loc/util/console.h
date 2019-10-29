#ifndef CONSOLE_H_MZIDGR2K
#define CONSOLE_H_MZIDGR2K

#include <ostream>
#include <rang.hpp>

namespace sens_loc { namespace util {
/// Tag-type for overload resolution of output operations to signal error
/// messages.
struct err {};
/// Colorized, bold output of "ERROR: " to signal error messages.
inline std::ostream& operator<<(std::ostream& os, err /*err_tag*/) {
    os << rang::fg::red << rang::style::bold << "ERROR:" << rang::style::reset
       << " ";
    return os;
}
/// Tag-type for overload resolution of output operations to signal warnings.
struct warn {};
/// Colorized, bold output of "WARN: " to signal warnings.
inline std::ostream& operator<<(std::ostream& os, warn /*warn_tag*/) {
    os << rang::fg::yellow << rang::style::bold << "WARN:" << rang::style::reset
       << " ";
    return os;
}
/// Tag-type for overload resolution of output operations to signal info output.
struct info {};
/// Colorized, bold output of "INFO: " to signal information messages.
inline std::ostream& operator<<(std::ostream& os, info /*info_tag*/) {
    os << rang::fg::reset << rang::style::bold << "INFO:" << rang::style::reset
       << " ";
    return os;
}
}}  // namespace sens_loc::util


#endif /* end of include guard: CONSOLE_H_MZIDGR2K */
