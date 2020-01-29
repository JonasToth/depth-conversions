#include "sens_loc/math/coordinate.h"
#include "sens_loc/math/pointcloud.h"

#include <sens_loc/camera_models/projection.h>

namespace sens_loc::camera_models {

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace sens_loc::math;
using namespace gsl;

imagepoints_t project_to_image(const pinhole<float>& pinhole,
                               const pointcloud_t&   points) noexcept {
    imagepoints_t pixel_coord;
    pixel_coord.reserve(points.size());

    for (const auto& pt : points)
        pixel_coord.emplace_back(pinhole.camera_to_pixel(pt));

    Ensures(pixel_coord.size() == points.size());
    return pixel_coord;
}

pointcloud_t project_to_sphere(const pinhole<float>& pinhole,
                               const imagepoints_t&  pixel) noexcept {
    pointcloud_t sphere_coord;
    sphere_coord.reserve(pixel.size());

    for (const auto& px : pixel)
        sphere_coord.emplace_back(1.0F * pinhole.pixel_to_sphere(px));

    Ensures(sphere_coord.size() == pixel.size());
    return sphere_coord;
}


imagepoints_t keypoint_to_coords(const vector<KeyPoint>& kps) noexcept {
    imagepoints_t pixel_coord(kps.size());
    Ensures(pixel_coord.size() == kps.size());

    for (size_t i = 0; i < kps.size(); ++i)
        pixel_coord[i] = math::pixel_coord<float>{kps[i].pt.x, kps[i].pt.y};

    return pixel_coord;
}

vector<KeyPoint> coords_to_keypoint(const math::imagepoints_t& pts) noexcept {
    vector<KeyPoint> kps;
    kps.reserve(pts.size());
    for (const auto& c : pts)
        kps.emplace_back(c.u(), c.v(), /*size=*/5.0F);

    Ensures(kps.size() == pts.size());
    return kps;
}

}  // namespace sens_loc::camera_models
