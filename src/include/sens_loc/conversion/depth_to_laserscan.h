#ifndef DEPTH_TO_LASERSCAN_H_P8V9HAVF
#define DEPTH_TO_LASERSCAN_H_P8V9HAVF

#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/image.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc { namespace conversion {

/// This function converts an orthographic depth image to an laser-scan like
/// depth image, aka. range-image.
///
/// Each pixel in the resulting laser-scan image describes the euclidean
/// distance from the camera center to the point in space.
/// The result type is 'Real'.
/// \tparam Real precision of the calculation
/// \tparam PixelType underlying type of the input image
/// \param depth_image orthgraphic depth image, e.g. from a kinect
/// \param intrinsic matching calibration of the sensor
/// \returns matrix with each value converted to the euclidean distance
/// \note invalid values (where the depth is zero) will be zero as well
/// \post each value is bigger or equal to the original pixel value
template <typename Real = float, typename PixelType = ushort>
math::image<Real>
depth_to_laserscan(const math::image<PixelType> &      depth_image,
                   const camera_models::pinhole<Real> &intrinsic) noexcept;

/// This function is the parallel implementation for the conversions.
/// \sa conversion::depth_to_laserscan
/// \param[in] depth_image,intrinsic same as in serial case
/// \param[out] out resulting converted image
/// \param[inout] flow taskgraph that will be used for the parallel jobs
/// \returns synchronization tasks before and after the conversion.
template <typename Real = float, typename PixelType = ushort>
std::pair<tf::Task, tf::Task>
par_depth_to_laserscan(const math::image<PixelType> &      depth_image,
                       const camera_models::pinhole<Real> &intrinsic,
                       math::image<Real> &out, tf::Taskflow &flow) noexcept;

namespace detail {
template <typename Real, typename PixelType>
void laserscan_inner(const int v, const math::image<PixelType> &depth_image,
                     const camera_models::pinhole<Real> &intrinsic,
                     math::image<Real> &                 euclid) {
    for (int u = 0; u < depth_image.data().cols; ++u) {
        const PixelType d_o = depth_image.at({u, v});
        euclid.at({u, v}) =
            orthografic_to_euclidian<Real>({u, v}, d_o, intrinsic);
    }
}
}  // namespace detail

template <typename Real, typename PixelType>
inline math::image<Real>
depth_to_laserscan(const math::image<PixelType> &      depth_image,
                   const camera_models::pinhole<Real> &intrinsic) noexcept {
    cv::Mat euclid(depth_image.data().rows, depth_image.data().cols,
                   math::detail::get_opencv_type<Real>());
    euclid = Real(0.);
    math::image<Real> euclid_image(std::move(euclid));

    for (int v = 0; v < depth_image.data().rows; ++v)
        detail::laserscan_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                 euclid_image);

    Ensures(euclid_image.data().rows == depth_image.data().rows);
    Ensures(euclid_image.data().cols == depth_image.data().cols);

    return euclid_image;
}


template <typename Real, typename PixelType>
inline std::pair<tf::Task, tf::Task>
par_depth_to_laserscan(const math::image<PixelType> &      depth_image,
                       const camera_models::pinhole<Real> &intrinsic,
                       math::image<Real> &out, tf::Taskflow &flow) noexcept {
    Expects(out.data().rows == depth_image.data().rows);
    Expects(out.data().cols == depth_image.data().cols);

    auto sync_points =
        flow.parallel_for(0, depth_image.data().rows, 1, [&](int v) {
            detail::laserscan_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                     out);
        });

    return sync_points;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_LASERSCAN_H_P8V9HAVF */
