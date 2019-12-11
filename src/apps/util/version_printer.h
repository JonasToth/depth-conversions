#include <string_view>

namespace sens_loc { namespace apps {

/// Helper Functor to print a consistent version information for each program.
struct print_version {
    print_version(std::string_view program_name)
        : program_name(program_name) {}
    [[noreturn]] void operator()(int /*count*/) noexcept;

  private:
    std::string_view program_name;
};
}}  // namespace sens_loc::apps
