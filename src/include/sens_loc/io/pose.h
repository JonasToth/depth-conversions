#ifndef POSE_H_ZQQXTH63
#define POSE_H_ZQQXTH63

#include <Eigen/Geometry>
#include <istream>
#include <optional>
#include <sstream>
#include <string>

namespace sens_loc::io {

/// Read a pose-file that gets procuded by the SfM-pipeline in the project.
///
/// The expected format is a human readable file with ASCII coding and the
/// following structure:
/// ```
/// r11 r12 r13 t1
/// r21 r22 r23 t2
/// r31 r32 r33 t3
/// ```
/// \param file_name path to the file to read from.
/// \returns an matrix usable for affine transformation on success, on failure
/// std::nullopt
inline std::optional<Eigen::Affine3f> load_pose(std::istream& in) {
    using std::ios_base;
    using std::nullopt;
    using std::string;
    using std::stringstream;

    if (!in.good())
        return nullopt;

    Eigen::Matrix4f p;
    p.fill(0.0F);
    p(3, 3) = 1.0F;

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
    if (std::abs(std::abs(p.block(0, 0, 3, 3).determinant()) - 1.0F) > 0.01F)
        return nullopt;

    return {Eigen::Affine3f(p)};
}
}  // namespace sens_loc::io

#endif /* end of include guard: POSE_H_ZQQXTH63 */
