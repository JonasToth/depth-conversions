#ifndef FILTER_FUNCTOR_H_ALM7TXD2
#define FILTER_FUNCTOR_H_ALM7TXD2

#include <sens_loc/math/image.h>
#include <sens_loc/preprocess/filter.h>

namespace sens_loc::apps {

/// Provides abstract interface for all filters.
/// \ingroup filter-driver
struct abstract_filter {
    virtual math::image<float>
    filter(const math::image<float>& input) const = 0;

    virtual ~abstract_filter() = default;
};

/// Apply the bilateral filter on depth images.
///
/// See
/// https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed
/// for more information.
/// \ingroup filter-driver
struct bilateral_filter : abstract_filter {
    bilateral_filter(double sigma_color, double sigma_space)
        : abstract_filter{}
        , _sigma_color{sigma_color}
        , _proximity{sigma_space} {
        Expects(sigma_color > 0.);
        Expects(sigma_space > 0.);
    }

    bilateral_filter(double sigma_color, int distance)
        : abstract_filter{}
        , _sigma_color{sigma_color}
        , _proximity{distance} {
        Expects(sigma_color > 0.);
        Expects(distance > 0);
    }

    math::image<float> filter(const math::image<float>& input) const override {
        return preprocess::bilateral_filter(input, _sigma_color, _proximity);
    }

  private:
    double                    _sigma_color;
    std::variant<int, double> _proximity;
};

/// Apply the median-blur with either 3 or 5 pixels diameter.
///
/// See
/// https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#ga564869aa33e58769b4469101aac458f9
/// for more information.
/// \ingroup filter-driver
struct median_blur_filter : abstract_filter {
    median_blur_filter(int kernel_size)
        : abstract_filter{}
        , _kernel_size{kernel_size} {
        Expects(kernel_size > 0);
        Expects(kernel_size % 2 == 1);  // Kernel size is odd.
    }

    math::image<float> filter(const math::image<float>& input) const override {
        return preprocess::median_blur(input, _kernel_size);
    }

  private:
    int _kernel_size;
};

}  // namespace sens_loc::apps

#endif /* end of include guard: FILTER_FUNCTOR_H_ALM7TXD2 */
