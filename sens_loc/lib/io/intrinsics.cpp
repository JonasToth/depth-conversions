#include <gsl/gsl>
#include <sens_loc/io/intrinsics.h>
#include <sstream>
#include <string>

namespace sens_loc::io {
std::optional<camera_models::pinhole> load_pinhole_intrinsic(std::istream &in) {
    if (!in.good())
        return std::nullopt;

    camera_models::pinhole p;
    double                 trash = 0.0;

    std::string line;
    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 1 with expected format 'w h'.
        ss >> p.w >> p.h;
        Expects(p.w > 0);
        Expects(p.h > 0);
        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 2 with expected format 'fx 0.0 cx'.
        ss >> p.fx >> trash >> p.cx;
        Expects(p.fx > 0.);
        Expects(p.cx > 0.);
        Expects(p.cx / static_cast<double>(p.w) < 0.75);
        Expects(p.cx / static_cast<double>(p.w) > 0.25);
        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }

    {
        std::getline(in, line);
        std::istringstream ss{line};
        // Parsing line 3 with expected format '0.0 fy cy'.
        ss >> trash >> p.fy >> p.cy;
        Expects(p.fy > 0.);
        Expects(p.cy > 0.);
        Expects(p.cy / static_cast<double>(p.h) < 0.75);
        Expects(p.cy / static_cast<double>(p.h) > 0.25);
        if (ss.rdstate() != std::ios_base::eofbit)
            return std::nullopt;
    }
    return p;
}
}  // namespace sens_loc::io
