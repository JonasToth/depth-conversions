#ifndef IMAGE_H_YQCE0AWR
#define IMAGE_H_YQCE0AWR

#include <gsl/gsl>
#include <opencv2/core/mat.hpp>
#include <sens_loc/math/coordinate.h>
#include <type_traits>

namespace sens_loc::math {

namespace detail {
template <typename Number>  // requires Number<Number>
inline int get_opencv_type() {
    static_assert(std::is_arithmetic_v<Number>);

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
    static_assert(std::is_arithmetic_v<PixelType>);

    image() = default;

    /// Construct the image from a \c cv::Mat and uphold the class invariant.
    explicit image(const cv::Mat& image) noexcept
        : _data(image) {
        Expects(image.type() == detail::get_opencv_type<PixelType>());
        Expects(image.channels() == 1);
        Expects(!image.empty());
    }
    image(const image<PixelType>& other) = default;

    // NOLINTNEXTLINE(performance-noexcept-move-constructor)
    image(image<PixelType>&& other) = default;
    explicit image(cv::Mat&& other) noexcept
        : _data(std::move(other)) {
        Expects(_data.type() == detail::get_opencv_type<PixelType>());
        Expects(_data.channels() == 1);
        Expects(!_data.empty());
    }

    image<PixelType>& operator=(const image<PixelType>& other) = default;
    image<PixelType>& operator=(const cv::Mat& other) noexcept {
        Expects(other.type() == detail::get_opencv_type<PixelType>());
        Expects(other.channels() == 1);
        Expects(!other.empty());
        _data = other;
        return *this;
    }

    // NOLINTNEXTLINE(performance-noexcept-move-constructor)
    image<PixelType>& operator=(image<PixelType>&& other) = default;
    image<PixelType>& operator=(cv::Mat&& other) noexcept {
        Expects(other.type() == detail::get_opencv_type<PixelType>());
        Expects(other.channels() == 1);
        Expects(!other.empty());
        _data = std::move(other);
        return *this;
    }

    ~image() = default;

    /// Return the width of the image.
    [[nodiscard]] int w() const noexcept { return _data.cols; }
    /// Return the height of the image.
    [[nodiscard]] int h() const noexcept { return _data.rows; }

    /// Read-Access in the image for some pixel \p p.
    template <typename Number = int>
    [[nodiscard]] PixelType at(const pixel_coord<Number>& p) const noexcept {
        static_assert(std::is_arithmetic_v<Number>);

        return _data.at<PixelType>(gsl::narrow_cast<int>(p.v()),
                                   gsl::narrow_cast<int>(p.u()));
    }
    /// Write-Access in the image for some pixel \p p.
    template <typename Number = int>
    [[nodiscard]] PixelType& at(const pixel_coord<Number>& p) noexcept {
        static_assert(std::is_arithmetic_v<Number>);

        return _data.at<PixelType>(gsl::narrow_cast<int>(p.v()),
                                   gsl::narrow_cast<int>(p.u()));
    }

    /// Get access to the underlying data to use it for normal cv operations.
    [[nodiscard]] const cv::Mat& data() const noexcept { return _data; }

  private:
    cv::Mat _data;
};

template <typename TargetType, typename PixelType>
image<TargetType> convert(const image<PixelType>& img) noexcept {
    static_assert(std::is_arithmetic_v<TargetType>);
    static_assert(std::is_arithmetic_v<PixelType>);

    if constexpr (std::is_same_v<TargetType, PixelType>)
        return img; // NOLINT(bugprone-suspicious-semicolon)

    cv::Mat tmp(img.h(), img.w(), detail::get_opencv_type<TargetType>());
    img.data().convertTo(tmp, detail::get_opencv_type<TargetType>());
    return math::image<TargetType>(std::move(tmp));
}

}  // namespace sens_loc::math

#endif /* end of include guard: IMAGE_H_YQCE0AWR */
