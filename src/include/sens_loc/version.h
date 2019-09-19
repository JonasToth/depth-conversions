#ifndef PROJECT_VERSION_H_IN_SXCFIRSV
#define PROJECT_VERSION_H_IN_SXCFIRSV

#include <string>

namespace sens_loc {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_MAJOR 0
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_MINOR 1
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_PATCH 0

inline std::string get_version() {
    return std::to_string(SENS_LOC_MAJOR) + "." +
           std::to_string(SENS_LOC_MINOR) + "." +
           std::to_string(SENS_LOC_PATCH);
}
inline int get_major_version() {
    return SENS_LOC_MAJOR;
}
inline int get_minor_version() {
    return SENS_LOC_MINOR;
}
inline int get_patch_version() {
    return SENS_LOC_PATCH;
}
}  // namespace sens_loc

#endif /* end of include guard: PROJECT_VERSION_H_IN_SXCFIRSV */
