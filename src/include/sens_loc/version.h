#ifndef PROJECT_VERSION_H_IN_SXCFIRSV
#define PROJECT_VERSION_H_IN_SXCFIRSV

#include <string>

/// Project namespace for all code related to sensor localization and depth
/// image conversion.
namespace sens_loc {

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_MAJOR 0
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_MINOR 4
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SENS_LOC_PATCH 0

/// \returns the string concatenation of the version numbers, following
/// https://semver.org/
inline std::string get_version() {
    return std::to_string(SENS_LOC_MAJOR) + "." +
           std::to_string(SENS_LOC_MINOR) + "." +
           std::to_string(SENS_LOC_PATCH);
}
/// \returns major version number, https://semver.org/
inline int get_major_version() {
    return SENS_LOC_MAJOR;
}
/// \returns minor version number, https://semver.org/
inline int get_minor_version() {
    return SENS_LOC_MINOR;
}
/// \returns patch version number, https://semver.org/
inline int get_patch_version() {
    return SENS_LOC_PATCH;
}

}  // namespace sens_loc

#endif /* end of include guard: PROJECT_VERSION_H_IN_SXCFIRSV */
