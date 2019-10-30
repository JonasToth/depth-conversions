#ifndef DEPTH_CLEAN_H_SVV42W9L
#define DEPTH_CLEAN_H_SVV42W9L

#include <opencv2/rgbd/depth.hpp>
#include <sens_loc/math/image.h>
#include <type_traits>

namespace sens_loc { namespace preprocess {

template <typename PixelType>
class depth_cleaner {
  public:
    static_assert(std::is_floating_point_v<PixelType>);

    explicit depth_cleaner(int window_size) noexcept
        : _cleaner(math::detail::get_opencv_type<PixelType>(), window_size) {
        Expects(window_size == 1 || window_size == 3 || window_size == 5 ||
                window_size == 7);
    }

    void operator()(const math::image<PixelType>& in,
                    math::image<PixelType>&       out) const noexcept {
        _cleaner(in.data(), out.data());
    }
    math::image<PixelType> operator()(const math::image<PixelType>& img) const
        noexcept {
        math::image<PixelType> out = img;
        _cleaner(img.data(), out.data());
        return out;
    }

  private:
    cv::rgbd::DepthCleaner _cleaner;
};

}}  // namespace sens_loc::preprocess

#endif /* end of include guard: DEPTH_CLEAN_H_SVV42W9L */
