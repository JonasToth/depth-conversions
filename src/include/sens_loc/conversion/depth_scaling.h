#ifndef DEPTH_SCALING_H_SHNMYLJV
#define DEPTH_SCALING_H_SHNMYLJV

#include <gsl/gsl>
#include <opencv2/core/mat.hpp>
#include <sens_loc/conversion/util.h>

namespace sens_loc { namespace conversion {

/// This function transforms a depth-image to a new depth-image.
/// The parameters are scaling and adding an offset.
//
/// \note This function saturates on overflow
/// \sa cv::Mat::convertTo
//
/// \param depth_image unchanged image that is scaled, can be anything, not
/// just depth_images. In this project it will be usually a depth image.
/// \param scale,offset scaling parameters
/// \returns matrix with each pixel: \f$scale * pixel + offset\f$
template <typename PixelType = ushort>
cv::Mat depth_scaling(const cv::Mat &depth_image, double scale,
                      double offset = 0.) noexcept {
    cv::Mat img(depth_image.rows, depth_image.cols,
                detail::get_cv_type<PixelType>());
    depth_image.convertTo(img, detail::get_cv_type<PixelType>(), scale, offset);
    return img;
}

}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_SCALING_H_SHNMYLJV */
