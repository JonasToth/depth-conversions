#ifndef DEPTH_SCALING_H_SHNMYLJV
#define DEPTH_SCALING_H_SHNMYLJV

#include <sens_loc/math/image.h>

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
math::image<PixelType> depth_scaling(const math::image<PixelType> &depth_image,
                                     double                        scale,
                                     double offset = 0.) noexcept {
    cv::Mat img(depth_image.h(), depth_image.w(),
                math::detail::get_opencv_type<PixelType>());
    depth_image.data().convertTo(
        img, math::detail::get_opencv_type<PixelType>(), scale, offset);
    return math::image<PixelType>(std::move(img));
}

}}  // namespace sens_loc::conversion

#endif /* end of include guard: DEPTH_SCALING_H_SHNMYLJV */
