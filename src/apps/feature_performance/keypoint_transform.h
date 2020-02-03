#ifndef KEYPOINT_TRANSFORM_H_ARXFM8VJ
#define KEYPOINT_TRANSFORM_H_ARXFM8VJ

#include <gsl/gsl>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/math/pointcloud.h>

namespace sens_loc::apps {

template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
math::pointcloud<Real>
keypoints_to_pointcloud(const std::vector<cv::KeyPoint>& kps,
                        const math::image<ushort>&       depth_image,
                        const Model<Real>&               c,
                        float                            unit_factor) noexcept {
    using camera_models::keypoint_to_coords;
    math::imagepoints<Real> pts   = keypoint_to_coords(kps);
    math::pointcloud<Real>  c_pts = project_to_sphere(c, pts);

    for (std::size_t i = 0; i < c_pts.size(); ++i) {
        auto orig_depth = depth_image.at(pts[i]);
        auto t_d        = unit_factor * gsl::narrow_cast<float>(orig_depth);
        c_pts[i]        = t_d * c_pts[i];
    }

    return c_pts;
}


/// Transform the pointcloud \c points_in_other_frame into the coordinate
/// frame of \c c and project these points into \c c's image plane.
/// \returns the backprojected points
/// \sa project_to_image
template <template <typename> typename Model = sens_loc::camera_models::pinhole,
          typename Real                      = float>
math::imagepoints<Real>
project_to_other_camera(const math::pose_t&           pose,
                        const math::pointcloud<Real>& points_in_other_frame,
                        const Model<Real>&            c) noexcept {
    math::pointcloud<Real>  transformed   = pose * points_in_other_frame;
    math::imagepoints<Real> backprojected = project_to_image(c, transformed);
    return backprojected;
}

}  // namespace sens_loc::apps

#endif /* end of include guard: KEYPOINT_TRANSFORM_H_ARXFM8VJ */
