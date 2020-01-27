#ifndef POINTCLOUD_H_8O5EBVHZ
#define POINTCLOUD_H_8O5EBVHZ

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gsl/gsl>
#include <sens_loc/math/image.h>

namespace sens_loc::math {

/// A pointcloud is defined as a "row-matrix" where each column is exactly one
/// point in homogenous coordinates.
using pointcloud_t = Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor>;

/// A set of points in homogeneous pixel coordinates.
using imagepoints_t = Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>;

/// Allocate a pointcloud with \c point_count number of points with homogeneous
/// coordinates.
/// \warning The points are uninitialized!
inline pointcloud_t make_pointcloud(unsigned int point_count) noexcept {
    pointcloud_t p(4, point_count);
    Ensures(p.cols() == point_count);
    Ensures(p.rows() == 4);
    // Homogeneous coordinates have '1.0' as the 4th element.
    p.row(3).fill(1.0F);
    return p;
}

/// Calculate the point-wise euclidean distance between each point in \c c0
/// and \c c1.
/// The result is a row-vector with the distance for point 'i' at position 'i'.
/// \pre c0.cols() == c1.cols() => the same number of points
/// \post each element in the result is non-negative.
inline Eigen::RowVectorXf pointwise_distance(const pointcloud_t& c0,
                                             const pointcloud_t& c1) noexcept {
    Expects(c0.cols() == c1.cols());

    pointcloud_t       diff      = c0 - c1;
    Eigen::RowVectorXf distances = diff.colwise().norm();

    Ensures(distances.rows() == 1);
    Ensures(distances.cols() == c0.cols());
    Ensures(distances.cols() == c1.cols());
    Ensures(distances.minCoeff() >= 0.0F);

    return distances;
}

/// Calculate the point-wise euclidean distance between each point in \c c0
/// and \c c1.
/// The result is a row-vector with the distance for point 'i' at position 'i'.
/// \pre c0.cols() == c1.cols() => the same number of points
/// \post each element in the result is non-negative.
inline Eigen::RowVectorXf pointwise_distance(const imagepoints_t& c0,
                                             const imagepoints_t& c1) noexcept {
    Expects(c0.cols() == c1.cols());
    Expects(c0.rows() == c1.rows());
    Expects(c0.rows() == 3);

    imagepoints_t      diff      = c0 - c1;
    Eigen::RowVectorXf distances = diff.colwise().norm();

    Ensures(distances.rows() == 1);
    Ensures(distances.cols() == c0.cols());
    Ensures(distances.cols() == c1.cols());
    Ensures(distances.minCoeff() >= 0.0F);

    return distances;
}

/// A pose is an affine transformation in 3 dimensions. Internally this is
/// processed with homogeneous coordinates.
/// \sa pointcloud_t
using pose_t = Eigen::Affine3f;

/// Calculate the relative pose to get from \c from to \c to.
///
/// Let \c O be the origin of a coordinate system, \c from and \c to are both
/// absolute poses in this coordinate system.
/// \c relative_pose calculates the relative pose to get from \c from to \c to.
/// \pre both poses need to be valid
/// \pre \c from must be invertable => pose is valid
inline pose_t relative_pose(const pose_t& from, const pose_t& to) noexcept {
    return from.inverse() * to;
}

}  // namespace sens_loc::math

#endif /* end of include guard: POINTCLOUD_H_8O5EBVHZ */
