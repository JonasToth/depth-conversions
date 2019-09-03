#ifndef DEPTH_TO_LASERSCAN_H_P8V9HAVF
#define DEPTH_TO_LASERSCAN_H_P8V9HAVF

#include <opencv2/imgcodecs.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/util.h>

namespace sens_loc { namespace conversion {
/// This function converts an orthographic depth image to an laser-scan like
/// depth image.
/// Each pixel in the resulting laser-scan image describes the euclidean
/// distance when projecting the pixel back into space.
/// The result type is 'Real'.
template <typename Real = float, typename PixelType = ushort>
cv::Mat depth_to_laserscan(const cv::Mat &               depth_image,
                           const camera_models::pinhole &intrinsic) noexcept {
    Expects(depth_image.channels() == 1);
    Expects(!depth_image.empty());
    Expects(depth_image.cols > 2);
    Expects(depth_image.rows > 2);

    using namespace detail;
    cv::Mat euclid(depth_image.rows, depth_image.cols, get_cv_type<Real>());

    for (int v = 0; v < depth_image.rows; ++v) {
        for (int u = 0; u < depth_image.cols; ++u) {
            const PixelType d_o = depth_image.at<PixelType>(v, u);
            euclid.at<Real>(v, u) =
                orthografic_to_euclidian<Real>(u, v, d_o, intrinsic);
        }
    }

    Ensures(euclid.rows == depth_image.rows);
    Ensures(euclid.cols == depth_image.cols);
    Expects(euclid.channels() == 1);
    Expects(euclid.type() == get_cv_type<Real>());

    return euclid;
}
}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_TO_LASERSCAN_H_P8V9HAVF */
