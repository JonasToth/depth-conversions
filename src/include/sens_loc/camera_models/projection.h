#ifndef PROJECTION_H_GUWYAV5U
#define PROJECTION_H_GUWYAV5U

#include "sens_loc/camera_models/concepts.h"

#include <opencv2/core/types.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/coordinate.h>
#include <sens_loc/math/pointcloud.h>

namespace sens_loc::camera_models {

/// Project the pointcloud \c points to pixel coordinates.
///
/// This function transforms a pointcloud with any camera model.
///
/// \tparam Model camera model implement with arbitrary precision
/// \tparam Real precision of calculation, camera model and resulting points
/// \pre \c points are in camera coordinates matching the camera position.
/// \pre \c intrinsic is a valid camera model
/// \post \c result are pixel coordinates
/// \post elements that are not in the image are pixel_coord{-1.0F, -1.0F}
/// \post project_to_sphere(result) gives the light ray direction for all
/// visible points.
/// \param intrinsic valid camera model
/// \param points set of points that will be projected into to pixel
/// coordinates.
/// \sa camera_to_pixel
/// \sa project_to_sphere
template <template <typename> typename Model = pinhole, typename Real = float>
math::imagepoints<Real>
project_to_image(const Model<Real>&            intrinsic,
                 const math::pointcloud<Real>& points) noexcept {
    static_assert(is_intrinsic_v<Model, Real>);

    math::imagepoints<Real> pixel_coord;
    pixel_coord.reserve(points.size());

    for (const auto& pt : points)
        pixel_coord.emplace_back(intrinsic.camera_to_pixel(pt));

    Ensures(pixel_coord.size() == points.size());
    return pixel_coord;
}

/// Project \c pixel coordinates to the unit-sphere.
//
/// \tparam Model camera model implement with arbitrary precision
/// \tparam Real precision of calculation, camera model and resulting points
/// \pre \c pixel are in pixel coordinates matching the intrinsic
/// \pre \c intrinsic is a valid camera model matching the pixel
/// \post \c result are camera coordinates, even though there are spherical by
/// their norm.
/// \post project_to_image(result) == pixel
/// \param intrinsic valid camera model
/// \param pixel set of points that will be projected to the unit sphere.
/// \sa project_to_image
template <template <typename> typename Model = pinhole, typename Real = float>
math::pointcloud<Real>
project_to_sphere(const Model<Real>&             intrinsic,
                  const math::imagepoints<Real>& pixel) noexcept {
    static_assert(is_intrinsic_v<Model, Real>);
    math::pointcloud<Real> sphere_coord;
    sphere_coord.reserve(pixel.size());

    for (const auto& px : pixel)
        sphere_coord.emplace_back(Real(1.0) * intrinsic.pixel_to_sphere(px));

    Ensures(sphere_coord.size() == pixel.size());
    return sphere_coord;
}

/// Convert keypoints to pixel coordinates.
template <typename Real = float>
math::imagepoints<Real>
keypoint_to_coords(const std::vector<cv::KeyPoint>& kps) noexcept {
    math::imagepoints<Real> pixel_coord(kps.size());
    Ensures(pixel_coord.size() == kps.size());

    for (size_t i = 0; i < kps.size(); ++i)
        pixel_coord[i] = math::pixel_coord<Real>(kps[i].pt.x, kps[i].pt.y);

    return pixel_coord;
}

/// Convert pixel coordinates back to keypoints to allow plotting with opencv.
/// \note Information like response, size and orientation are lost set to dummy
/// values and not stored in \c imagepoints_t.
template <typename Real = float>
std::vector<cv::KeyPoint>
coords_to_keypoint(const math::imagepoints<Real>& pts) noexcept {
    std::vector<cv::KeyPoint> kps;
    kps.reserve(pts.size());
    for (const auto& c : pts)
        kps.emplace_back(gsl::narrow_cast<float>(c.u()),
                         gsl::narrow_cast<float>(c.v()), /*size=*/5.0F);

    Ensures(kps.size() == pts.size());
    return kps;
}
}  // namespace sens_loc::camera_models

#endif /* end of include guard: PROJECTION_H_GUWYAV5U */
