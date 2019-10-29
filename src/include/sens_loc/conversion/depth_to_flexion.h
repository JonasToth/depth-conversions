#ifndef DEPTH_TO_TRIPLE_H_Y021ENVZ
#define DEPTH_TO_TRIPLE_H_Y021ENVZ

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>
#include <sens_loc/math/eigen_types.h>
#include <sens_loc/math/image.h>
#include <taskflow/taskflow.hpp>

namespace sens_loc { namespace conversion {

/// Convert range image to a flexion-image.
///
/// The flexion image is a derived image type that describes the angle between
/// the normals of the surface patch at pixel (u,v). The normals are estimated
/// with the direct neighbours and and the diagonal neighbours.
/// This is a measure of curvature as well.
///
/// \tparam Real precision of the calculation
/// \tparam PixelType underlying type of \p depth_image
/// \tparam Intrinsic camera model that projects pixel to the unit sphere
/// \param depth_image range image
/// \param intrinsic calibration of the sensor that took the image
/// \returns flexion image, each pixel in the range \f$[0,1]\f$
/// \sa conversion::depth_to_laserscan
/// \pre \p depth_image is not empty
/// \pre intrinsic matches the sensor that took the image
/// \sa camera_models::is_intrinsic_v
template <typename Real = float, typename PixelType = float,
          template <typename> typename Intrinsic = camera_models::pinhole>
math::image<Real> depth_to_flexion(const math::image<PixelType> &depth_image,
                                   const Intrinsic<Real> &intrinsic) noexcept;

/// Convert range image to a flexion image in parallel
//
/// This function implements the same functionality but with row-parallelism.
//
/// \sa depth_to_flexion
/// \param[in] depth_image,intrinsic same as in \p depth_to_flexion
/// \param[out] flexion_image output image
/// \param[inout] flow taskgraph the calculations will be registered in
/// \returns synchronization task before and after the calculation
/// \note the calculation does not happen instantly but first a taskgraph is
/// \pre \p flexion_image has the same dimension as \p depth_image
/// \pre the underlying types match the defined template parameters
/// built. This graph will then execute all the tasks on request.
template <typename Real = float, typename PixelType = float,
          template <typename> typename Intrinsic = camera_models::pinhole>
std::pair<tf::Task, tf::Task> par_depth_to_flexion(
    const math::image<PixelType> &depth_image, const Intrinsic<Real> &intrinsic,
    math::image<Real> &flexion_image, tf::Taskflow &flow) noexcept;

/// Scale the flexion image to \p PixelType for normal image visualization.
///
/// This function simply scales the image to the full possible range of
/// \p PixelType.
/// \sa cv::Mat::convertTo
/// \pre underlying types match
/// \pre range of each pixel is \f$[0, 1]\f$
/// \post range of result pixels is \f$[PixelType_{min}, PixelType_{max}]\f$
template <typename Real, typename PixelType>
math::image<PixelType>
convert_flexion(const math::image<Real> &flexion_image) noexcept;


namespace detail {
using ::sens_loc::math::vec;
template <typename Real = float, template <typename> typename Intrinsic>
math::camera_coord<Real> to_camera(const Intrinsic<Real> &       intrinsic,
                                   const math::pixel_coord<int> &p,
                                   Real                          d) noexcept {
    const math::sphere_coord<Real> P_s = intrinsic.pixel_to_sphere(p);
    return math::camera_coord<Real>(d * P_s.Xs(), d * P_s.Ys(), d * P_s.Zs());
}

template <typename Real = float, typename PixelType = float,
          template <typename> typename Intrinsic>
inline void flexion_inner(int v, const math::image<PixelType> &depth_image,
                          const Intrinsic<Real> &intrinsic,
                          math::image<Real> &    out) {
    for (int u = 1; u < depth_image.w() - 1; ++u) {
        const Real d__1__0 = depth_image.at({u, v - 1});
        const Real d_1__0  = depth_image.at({u, v + 1});

        const Real d__0__1 = depth_image.at({u - 1, v});
        const Real d__0_1  = depth_image.at({u + 1, v});

        const Real d__1_1 = depth_image.at({u + 1, v - 1});
        const Real d_1__1 = depth_image.at({u - 1, v + 1});

        const Real d__1__1 = depth_image.at({u - 1, v - 1});
        const Real d_1_1   = depth_image.at({u + 1, v + 1});

        // If any of the depths is zero, the resulting vector will be the
        // null vector. This with then propagate through as zero and does not
        // induce any undefined behaviour or other problems.
        // Not short-circuiting results in easier vectorization / GPU
        // acceleration.

        using math::camera_coord;
        using math::pixel_coord;

        const camera_coord<Real> surface_pt0 =
            to_camera(intrinsic, {u, v - 1}, d__1__0);
        const camera_coord<Real> surface_pt1 =
            to_camera(intrinsic, {u, v + 1}, d_1__0);
        const camera_coord<Real> surface_dir0 = surface_pt1 - surface_pt0;

        const camera_coord<Real> surface_pt2 =
            to_camera(intrinsic, {u - 1, v}, d__0__1);
        const camera_coord<Real> surface_pt3 =
            to_camera(intrinsic, {u + 1, v}, d__0_1);
        const camera_coord<Real> surface_dir1 = surface_pt3 - surface_pt2;

        const camera_coord<Real> surface_pt4 =
            to_camera(intrinsic, {u + 1, v - 1}, d__1_1);
        const camera_coord<Real> surface_pt5 =
            to_camera(intrinsic, {u - 1, v + 1}, d_1__1);
        const camera_coord<Real> surface_dir2 = surface_pt5 - surface_pt4;

        const camera_coord<Real> surface_pt6 =
            to_camera(intrinsic, {u - 1, v - 1}, d__1__1);
        const camera_coord<Real> surface_pt7 =
            to_camera(intrinsic, {u + 1, v + 1}, d_1_1);
        const camera_coord<Real> surface_dir3 = surface_pt7 - surface_pt6;

        const auto cross0 =
            surface_dir0.normalized().cross(surface_dir1.normalized());
        const auto cross1 =
            surface_dir2.normalized().cross(surface_dir3.normalized());

        const auto flexion = std::abs(cross0.dot(cross1));

        Ensures(flexion >= 0.);
        Ensures(flexion <= 1.);

        out.at({u, v}) = flexion;
    }
}

}  // namespace detail

template <typename Real, typename PixelType,
          template <typename> typename Intrinsic>
inline math::image<Real>
depth_to_flexion(const math::image<PixelType> &depth_image,
                 const Intrinsic<Real> &       intrinsic) noexcept {
    static_assert(camera_models::is_intrinsic_v<Intrinsic, Real>);

    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());

    cv::Mat flexion(depth_image.h(), depth_image.w(),
                    math::detail::get_opencv_type<Real>());
    flexion = Real(0.);
    math::image<Real> flexion_image(std::move(flexion));
    for (int v = 1; v < depth_image.h() - 1; ++v)
        detail::flexion_inner<Real, PixelType>(v, depth_image, intrinsic,
                                               flexion_image);

    Ensures(flexion_image.w() == depth_image.w());
    Ensures(flexion_image.h() == depth_image.h());

    return flexion_image;
}

/// Convert an euclidian depth image to a flexion-image.
template <typename Real, typename PixelType,
          template <typename> typename Intrinsic>
inline std::pair<tf::Task, tf::Task> par_depth_to_flexion(
    const math::image<PixelType> &depth_image, const Intrinsic<Real> &intrinsic,
    math::image<Real> &flexion_image, tf::Taskflow &flow) noexcept {
    static_assert(camera_models::is_intrinsic_v<Intrinsic, Real>);

    Expects(depth_image.w() == intrinsic.w());
    Expects(depth_image.h() == intrinsic.h());

    Expects(flexion_image.w() == depth_image.w());
    Expects(flexion_image.h() == depth_image.h());

    auto sync_points = flow.parallel_for(
        1, depth_image.h() - 1, 1, [&](int v) noexcept {
            detail::flexion_inner<Real, PixelType>(v, depth_image, intrinsic,
                                                   flexion_image);
        });

    return sync_points;
}

template <typename Real, typename PixelType>
inline math::image<PixelType>
convert_flexion(const math::image<Real> &flexion_image) noexcept {
    cv::Mat img(flexion_image.h(), flexion_image.w(),
                math::detail::get_opencv_type<PixelType>());

    const Real scale = Real(std::numeric_limits<PixelType>::max()) -
                       Real(std::numeric_limits<PixelType>::min());
    const Real offset = std::numeric_limits<PixelType>::min();

    flexion_image.data().convertTo(
        img, math::detail::get_opencv_type<PixelType>(), scale, offset);

    Ensures(img.cols == flexion_image.w());
    Ensures(img.rows == flexion_image.h());
    Ensures(img.type() == math::detail::get_opencv_type<PixelType>());
    Ensures(img.channels() == 1);

    return math::image<PixelType>(std::move(img));
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_TRIPLE_H_Y021ENVZ */
