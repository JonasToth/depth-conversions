#ifndef FILTER_FUNCTOR_H_ALM7TXD2
#define FILTER_FUNCTOR_H_ALM7TXD2

#include <sens_loc/math/image.h>
#include <sens_loc/preprocess/filter.h>

namespace sens_loc::apps {

struct abstract_filter {
    virtual math::image<float>
    filter(const math::image<float>& input) const = 0;

    virtual ~abstract_filter() = default;
};

struct bilateral_filter : abstract_filter {
    bilateral_filter(double sigma_color, double sigma_space)
        : abstract_filter{}
        , _sigma_color{sigma_color}
        , _proximity{sigma_space} {}

    math::image<float> filter(const math::image<float>& input) const override {
        return preprocess::bilateral_filter(input, _sigma_color, _proximity);
    }

  private:
    double                    _sigma_color;
    std::variant<int, double> _proximity;
};

struct median_blur_filter : abstract_filter {
    median_blur_filter(int kernel_size)
        : abstract_filter{}
        , _kernel_size{kernel_size} {}

    math::image<float> filter(const math::image<float>& input) const override {
        return preprocess::median_blur(input, _kernel_size);
    }

  private:
    int _kernel_size;
};

}  // namespace sens_loc::apps

#endif /* end of include guard: FILTER_FUNCTOR_H_ALM7TXD2 */
