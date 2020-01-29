#ifndef POSE_H_ZQQXTH63
#define POSE_H_ZQQXTH63

#include <sens_loc/math/pointcloud.h>
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
/// \param in input stream that contains the pose
/// \returns an matrix usable for affine transformation on success, on failure
/// std::nullopt
std::optional<math::pose_t> load_pose(std::istream& in) noexcept;

}  // namespace sens_loc::io

#endif /* end of include guard: POSE_H_ZQQXTH63 */
