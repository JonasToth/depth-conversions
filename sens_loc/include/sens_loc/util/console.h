#ifndef CONSOLE_H_MZIDGR2K
#define CONSOLE_H_MZIDGR2K

#include <ostream>
#include <rang.hpp>

namespace sens_loc { namespace util {
struct err {};
std::ostream &operator<<(std::ostream &os, err /*err_tag*/) {
    os << rang::fg::red << rang::style::bold << "ERROR:" << rang::style::reset
       << " ";
    return os;
}
struct warn {};
std::ostream &operator<<(std::ostream &os, warn /*warn_tag*/) {
    os << rang::fg::yellow << rang::style::bold << "WARN:" << rang::style::reset
       << " ";
    return os;
}
struct info {};
std::ostream &operator<<(std::ostream &os, info /*info_tag*/) {
    os << rang::fg::reset << rang::style::bold << "INFO:" << rang::style::reset
       << " ";
    return os;
}
}}  // namespace sens_loc::util


#endif /* end of include guard: CONSOLE_H_MZIDGR2K */
