#ifndef PROJECTION_H_GUWYAV5U
#define PROJECTION_H_GUWYAV5U

#include <opencv2/core/types.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/pointcloud.h>

namespace sens_loc::camera_models {

/// Project the pointcloud \c points to pixel coordinates.
/// This function trades computational efficiency (direct matrix
/// multiplications) with the typesafety the individual coordinate-types
/// provide. All compuations are done with 'float' precision.
/// \pre \c points are in camera coordinates matching the camera position.
/// \pre \c pinhole is a valid pinhole model
/// \post \c result are homogeneous pixel coordinates
/// \post elements that are no in the image are {0.0F, 0.0F, 0.0F}
/// \param pinhole valid pinhole model without distortion
/// \param points set of points that will be projected into the image.
math::imagepoints_t project_to_image(const pinhole<float>&     pinhole,
                                     const math::pointcloud_t& points) noexcept;

/// Project pixel coordinates to the unit-sphere.
/// \sa project_to_image
math::pointcloud_t
project_to_sphere(const pinhole<float>&   pinhole,
                  const Eigen::Matrix3Xf& homogeneous_pixel) noexcept;

/// Convert keypoints to homogeneous pixel coordinates.
math::imagepoints_t
keypoint_to_coords(const std::vector<cv::KeyPoint>& kps) noexcept;

/// Convert homogeneous pixel coordinates back to keypoints to allow plotting
/// with opencv.
/// \note Information like response, size and orientation are lost!
std::vector<cv::KeyPoint>
coords_to_keypoint(const math::imagepoints_t& pts) noexcept;

}  // namespace sens_loc::camera_models

#endif /* end of include guard: PROJECTION_H_GUWYAV5U */
