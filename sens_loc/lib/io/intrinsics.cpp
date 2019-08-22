#include <sens_loc/io/intrinsics.h>
#include <sstream>
#include <string>

namespace sense_loc { namespace io {
std::optional<sens_loc::io::pinhole_parameters>
load_pinhole_intrinsic(std::istream &in) {
    double fx    = 0.0;
    double fy    = 0.0;
    double cx    = 0.0;
    double cy    = 0.0;
    double trash = 0.0;

    std::string line;
    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 1 with expected format 'fx 0.0 cx'.
        ss >> fx >> trash >> cx;
        if (!ss.good())
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 2 with expected format '0.0 fy cy'.
        ss >> trash >> fy >> cy;
        if (!ss.good())
            return std::nullopt;
    }
    return {{fx, fy, cx, cy}};
}
}}  // namespace sense_loc::io
