#include <iostream>
#include <sens_loc/io/pose.h>
#include <sstream>
#include <string>

namespace sens_loc::io {
std::optional<math::pose_t> load_pose(std::istream& in) noexcept {
    using std::ios_base;
    using std::nullopt;
    using std::string;
    using std::stringstream;

    if (!in.good())
        return nullopt;

    Eigen::Matrix4f p = Eigen::Matrix4f::Identity();

    int line_nbr = 0;
    for (string line; getline(in, line) && line_nbr < 3; ++line_nbr) {
        stringstream ss(line);
        ss >> p(line_nbr, 0) >> p(line_nbr, 1) >> p(line_nbr, 2) >>
            p(line_nbr, 3);

        if (ss.fail() || ss.rdstate() != ios_base::eofbit)
            return nullopt;
    }

    // There is at least one row missing.
    if (line_nbr != 3)
        return nullopt;

    // The affine transformation is not only a rotation and translation, but
    // includes shearing. This is not allowed for a pose!
    // Note: arguments are: 'block(startRow, startCol, blockRows, blockCols)'
    if (std::abs(std::abs(p.block<3, 3>(0, 0).determinant()) - 1.0F) >
        0.0001F) {
        std::cerr << "Bad Rotation matrix\n";
        return nullopt;
    }

    return p;
}
}  // namespace sens_loc::io
