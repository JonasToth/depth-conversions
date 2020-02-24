#ifndef CONSOLE_H_MZIDGR2K
#define CONSOLE_H_MZIDGR2K

#include <mutex>
#include <ostream>
#include <rang.hpp>

namespace sens_loc {
class synced {
  public:
    synced() { _stdio_mutex.lock(); }

    synced(const synced&) = delete;
    synced(synced&&)      = delete;
    synced& operator=(synced&&) = delete;
    synced& operator=(const synced&) = delete;

    ~synced() { _stdio_mutex.unlock(); }

  private:
    static std::mutex _stdio_mutex;
};
}  // namespace sens_loc

namespace sens_loc::util {

/// Tag-Type to clear the line with a VT-100 terminal escape sequence.
struct clear_line {};
inline std::ostream& operator<<(std::ostream& os,
                                clear_line /*clear_tag*/) noexcept {
    if (rang::rang_implementation::isTerminal(os.rdbuf()))
        os << "\033[2K\r";
    else
        os << "\n";
    return os;
}

/// Tag-type for overload resolution of output operations to signal error
/// messages.
struct err {};
/// Colorized, bold output of "ERROR: " to signal error messages.
inline std::ostream& operator<<(std::ostream& os, err /*err_tag*/) noexcept {
    os << clear_line{} << rang::fg::red << rang::style::bold
       << "ERROR:" << rang::style::reset << " ";
    return os;
}
/// Tag-type for overload resolution of output operations to signal warnings.
struct warn {};
/// Colorized, bold output of "WARN: " to signal warnings.
inline std::ostream& operator<<(std::ostream& os, warn /*warn_tag*/) noexcept {
    os << clear_line{} << rang::fg::yellow << rang::style::bold
       << "WARN:" << rang::style::reset << " ";
    return os;
}
/// Tag-type for overload resolution of output operations to signal info output.
struct info {};
/// Colorized, bold output of "INFO: " to signal information messages.
inline std::ostream& operator<<(std::ostream& os, info /*info_tag*/) noexcept {
    os << clear_line{} << rang::fg::reset << rang::style::bold
       << "INFO:" << rang::style::reset << " ";
    return os;
}
}  // namespace sens_loc::util


#endif /* end of include guard: CONSOLE_H_MZIDGR2K */
