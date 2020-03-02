#ifndef COMMON_STRUCTURES_H_R5TFSBX6
#define COMMON_STRUCTURES_H_R5TFSBX6

#include <gsl/gsl>
#include <string_view>

namespace sens_loc::util {

/// This data is common for all functionality that reads in files in
/// batch-processing mode.
struct processing_input {
    std::string_view input_pattern;
    int              start;
    int              end;

    processing_input(std::string_view input_pattern,
                     int              start,
                     int              end) noexcept
        : input_pattern{input_pattern}
        , start{start}
        , end{end} {
        Expects(!input_pattern.empty());
        Expects(start <= end);
    }
};

}  // namespace sens_loc::util

#endif /* end of include guard: COMMON_STRUCTURES_H_R5TFSBX6 */
