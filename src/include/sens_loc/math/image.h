#ifndef IMAGE_H_YQCE0AWR
#define IMAGE_H_YQCE0AWR

#include <gsl/gsl>
#include <opencv2/core/mat.hpp>
#include <sens_loc/math/coordinate.h>

namespace sens_loc { namespace math {

namespace detail {
template <typename Number>  // requires Number<Number>
inline int get_opencv_type() {
    if constexpr (std::is_same<Number, float>::value)
        return CV_32F;  // NOLINT(bugprone-branch-clone)
    else if constexpr (std::is_same<Number, double>::value)
        return CV_64F;
    else if constexpr (std::is_same<Number, uchar>::value)
        return CV_8U;
    else if constexpr (std::is_same<Number, schar>::value)
        return CV_8S;
    else if constexpr (std::is_same<Number, ushort>::value)
        return CV_16U;
    else if constexpr (std::is_same<Number, short>::value)
        return CV_16S;
    else
        return -1;
}
}  // namespace detail

/// This function encapsulates an \c cv::Mat and ensures the common
/// preconditions for this code base are enforced.
///
/// \invariant the image has only 1 channel.
/// \invariant the underlying data types are consistent
/// \invariant access is only done in \p pixel_coord<int>
///
/// \tparam PixelType the underlying type of the \c cv::Mat
/// \sa math::pixel_coord
template <typename PixelType = ushort>
class image {
  public:
    image() = default;

    explicit image(const cv::Mat &image) noexcept
        : data(image) {
        Expects(image.type() == detail::get_opencv_type<PixelType>());
        Expects(image.channels() == 1);
        Expects(!image.empty());
    }
    image(const image<PixelType> &other) = default;

    image(image<PixelType> &&other) = default;
    image(cv::Mat &&other) noexcept
        : data(std::move(other)) {
        Expects(data.type() == detail::get_opencv_type<PixelType>());
        Expects(data.channels() == 1);
        Expects(!data.empty());
    }

    image<PixelType> &operator=(const image<PixelType> &other) = default;
    image<PixelType> &operator=(const cv::Mat &other) noexcept {
        Expects(other.type() == detail::get_opencv_type<PixelType>());
        Expects(other.channels() == 1);
        Expects(!other.empty());
        data = other;
        return *this;
    }

    image<PixelType> &operator=(image<PixelType> &&other) = default;
    image<PixelType> &operator=(cv::Mat &&other) noexcept {
        Expects(other.type() == detail::get_opencv_type<PixelType>());
        Expects(other.channels() == 1);
        Expects(!other.empty());
        data = std::move(other);
        return *this;
    }

    ~image() = default;

    template <typename Number = int>
    [[nodiscard]] PixelType at(const pixel_coord<Number> &p) const noexcept {
        return data.at<PixelType>(gsl::narrow_cast<int>(p.v()),
                                  gsl::narrow_cast<int>(p.u()));
    }
    template <typename Number = int>
    [[nodiscard]] PixelType &at(const pixel_coord<Number> &p) noexcept {
        return data.at<PixelType>(gsl::narrow_cast<int>(p.v()),
                                  gsl::narrow_cast<int>(p.u()));
    }

  private:
    cv::Mat data;
};

}}  // namespace sens_loc::math

#endif /* end of include guard: IMAGE_H_YQCE0AWR */
