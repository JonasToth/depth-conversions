#ifndef POSE_H_ZQQXTH63
#define POSE_H_ZQQXTH63

#include <Eigen/Geometry>
#include <istream>
#include <optional>

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
std::optional<Eigen::Affine3f> load_pose(std::istream& in) noexcept;

}  // namespace sens_loc::io

#endif /* end of include guard: POSE_H_ZQQXTH63 */
