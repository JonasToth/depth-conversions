#include "converters.h"

namespace sens_loc::apps {

#define INSTANTIATE_CONVERTER(type)                                            \
    template class type##_converter<camera_models::pinhole<double>>;           \
    template class type##_converter<camera_models::equirectangular<double>>;

INSTANTIATE_CONVERTER(range)
INSTANTIATE_CONVERTER(bearing)
INSTANTIATE_CONVERTER(gauss_curv)
INSTANTIATE_CONVERTER(mean_curv)
INSTANTIATE_CONVERTER(max_curve)
INSTANTIATE_CONVERTER(flexion)

}  // namespace sens_loc::apps
