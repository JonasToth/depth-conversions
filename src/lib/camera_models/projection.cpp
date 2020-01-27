#include "sens_loc/math/pointcloud.h"

#include <sens_loc/camera_models/projection.h>

namespace sens_loc::camera_models {

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace sens_loc::math;
using namespace gsl;

static Matrix<float, 3, 3> camera_matrix(const pinhole<float>& p) noexcept {
    Matrix<float, 3, 3> camera_matrix = Matrix<float, 3, 3>::Identity();
    camera_matrix(0, 0)               = p.fx();
    camera_matrix(1, 1)               = p.fy();
    camera_matrix(0, 2)               = p.cx();
    camera_matrix(1, 2)               = p.cy();
    return camera_matrix;
}

imagepoints_t project_to_image(const pinhole<float>& pinhole,
                               const pointcloud_t&   points) noexcept {
    Matrix<float, 3, 3> K = camera_matrix(pinhole);

    imagepoints_t image_coord(3, points.cols());

    for (int i = 0; i < points.cols(); ++i) {
        const float Z = points(2, i);
        // A point in the image-plane can not be projected by the pinhole model.
        // Such points are marked as invalid.
        image_coord.col(i) =
            Z != 0.0F
                ? Vector2f{points(0, i) / Z, points(1, i) / Z}.homogeneous()
                : Vector3f{0.0F, 0.0F, 0.0F};
    }

    imagepoints_t pixel_coord = K * image_coord;

    // Mark every pixel-coordinate that is not within the viewport as invalid.
    for (int i = 0; i < pixel_coord.cols(); ++i) {
        Vector3f pixel = pixel_coord.col(i);
        if ((pixel.x() < 0.0F || pixel.x() > narrow_cast<float>(pinhole.w())) ||
            (pixel.y() < 0.0F || pixel.y() > narrow_cast<float>(pinhole.h())))
            pixel_coord.col(i) = Vector3f{0.0F, 0.0F, 0.0F};
    }

    Ensures(pixel_coord.cols() == points.cols());
    Ensures(pixel_coord.rows() == 3);

    return pixel_coord;
}

pointcloud_t project_to_sphere(const pinhole<float>& pinhole,
                               const imagepoints_t&  pixel) noexcept {
    Matrix<float, 3, 3> K_inv = camera_matrix(pinhole).inverse();

    imagepoints_t image_coord = K_inv * pixel;
    pointcloud_t  sphere_coord(4, pixel.cols());

    for (int i = 0; i < pixel.cols(); ++i) {
        const float    f      = 1.0F / image_coord.col(i).norm();
        const Vector3f sphere = f * image_coord.col(i);
        sphere_coord.col(i)   = sphere.homogeneous();
    }

    Ensures(sphere_coord.rows() == 4);
    Ensures(sphere_coord.cols() == pixel.cols());
    Ensures(sphere_coord.row(3).sum() == narrow_cast<float>(pixel.cols()));
    return sphere_coord;
}


imagepoints_t keypoint_to_coords(const vector<KeyPoint>& kps) noexcept {
    imagepoints_t pixel_coord(3, narrow_cast<int>(kps.size()));

    for (size_t i = 0; i < kps.size(); ++i)
        pixel_coord.col(i) = Vector2f{kps[i].pt.x, kps[i].pt.y}.homogeneous();

    Ensures(pixel_coord.cols() == kps.size());
    Ensures(pixel_coord.rows() == 3);
    return pixel_coord;
}

vector<KeyPoint> coords_to_keypoint(const math::imagepoints_t& pts) noexcept {
    vector<KeyPoint> kps(pts.cols());
    for (int i = 0; i < pts.cols(); ++i)
        kps[i].pt = {pts.col(i).x(), pts.col(i).y()};

    Ensures(kps.size() == std::size_t(pts.cols()));
    return kps;
}

}  // namespace sens_loc::camera_models
