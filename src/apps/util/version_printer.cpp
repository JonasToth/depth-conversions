#include "version_printer.h"

#include <cstdlib>
#include <iostream>
#include <sens_loc/version.h>

namespace sens_loc::apps {

[[noreturn]] void print_version::operator()(int /*count*/) noexcept {
    std::cout << program_name << " v" << get_version() << "\n";
    std::exit(EXIT_SUCCESS);
}

}  // namespace sens_loc::apps
