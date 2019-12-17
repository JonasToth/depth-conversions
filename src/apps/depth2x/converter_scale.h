#ifndef CONVERTER_SCALE_H_EMX4RQOH
#define CONVERTER_SCALE_H_EMX4RQOH

#include <util/batch_converter.h>

namespace sens_loc::apps {

/// \addtogroup conversion-driver
/// @{

/// Scale depth images and add a constant value to each pixel.
/// \sa conversion::depth_scaling
class scale_converter : public batch_converter {
  public:
    scale_converter(const file_patterns& files, double scale, double offset)
        : batch_converter(files)
        , _scale{scale}
        , _offset{offset} {}
    scale_converter(const scale_converter&) = default;
    scale_converter(scale_converter&&)      = default;
    scale_converter& operator=(const scale_converter&) = default;
    scale_converter& operator=(scale_converter&&) = default;
    ~scale_converter() override                   = default;

  private:
    double _scale  = 1.0;
    double _offset = 0.0;

    [[nodiscard]] bool process_file(const math::image<float>& depth_image,
                                    int idx) const noexcept override;
};
/// @}
}  // namespace sens_loc::apps

#endif /* end of include guard: CONVERTER_SCALE_H_EMX4RQOH */
