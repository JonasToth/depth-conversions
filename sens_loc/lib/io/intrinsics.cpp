#include <iostream>
#include <sens_loc/io/intrinsics.h>
#include <sstream>
#include <string>

namespace sens_loc { namespace io {
std::optional<pinhole_parameters> load_pinhole_intrinsic(std::istream &in) {
    if (!in.good())
        return std::nullopt;

    pinhole_parameters p;
    double             trash = 0.0;

    std::string line;
    {
        std::getline(in, line);
        std::cout << line << "\n";
        std::istringstream ss{line};
        // Parsing line 1 with expected format 'fx 0.0 cx'.
        ss >> p.fx >> trash >> p.cx;
        std::cout << p.fx << "\n" << trash << "\n" << p.cx << "\n";
        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::cout << line << "\n";
        std::istringstream ss{line};
        // Parsing line 2 with expected format '0.0 fy cy'.
        ss >> trash >> p.fy >> p.cy;
        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }
    return p;
}
}}  // namespace sens_loc::io
