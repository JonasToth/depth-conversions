#include "version_printer.h"

#include <iostream>
#include <sens_loc/version.h>

namespace sens_loc::apps {

[[noreturn]] void print_version::operator()(int /*count*/) noexcept {
    std::cout << program_name << " " << get_version() << "\n";
    exit(0);
}

}  // namespace sens_loc::apps
