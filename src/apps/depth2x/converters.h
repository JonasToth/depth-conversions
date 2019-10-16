#ifndef CONVERTERS_H_HVFGCFVK
#define CONVERTERS_H_HVFGCFVK

#include "batch_converter.h"

#include <gsl/gsl>

namespace sens_loc { namespace apps {

/// \addtogroup conversion-driver
/// @{

/// Batch conversion to bearing-angle images.
/// \sa conversion::depth_to_bearing
class bearing_converter : public batch_pinhole_converter {
  public:
    bearing_converter(const file_patterns &files, depth_type t,
                      const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        if (files.horizontal.empty() && files.vertical.empty() &&
            files.diagonal.empty() && files.antidiagonal.empty()) {
            throw std::invalid_argument{
                "Missing output pattern for at least one bearing direction"};
        }
    }
    bearing_converter(const bearing_converter &) = default;
    bearing_converter(bearing_converter &&)      = default;
    bearing_converter &operator=(const bearing_converter &) = default;
    bearing_converter &operator=(bearing_converter &&) = default;
    ~bearing_converter() override                      = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;
};

/// Convert orthographic depth-images to range (laserscan-like) images.
/// \sa conversion::depth_to_laserscan
class range_converter : public batch_pinhole_converter {
  public:
    range_converter(const file_patterns &files, depth_type t,
                    const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {}
    range_converter(const range_converter &) = default;
    range_converter(range_converter &&)      = default;
    range_converter &operator=(const range_converter &) = default;
    range_converter &operator=(range_converter &&) = default;
    ~range_converter() override                    = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;
};

/// Convert range-images to gaussian curvature images.
/// \sa conversion::depth_to_gaussian_curvature
class gauss_curv_converter : public batch_pinhole_converter {
  public:
    /// \param files,t,intrinsic normal parameters for batch conversion
    /// \param lower_bound,upper_bound clamping parameters. Values below/above
    /// will map to these values.
    gauss_curv_converter(const file_patterns &files, depth_type t,
                         const camera_models::pinhole &intrinsic,
                         double lower_bound, double upper_bound)
        : batch_pinhole_converter(files, t, intrinsic)
        , lower_bound{lower_bound}
        , upper_bound{upper_bound} {}
    gauss_curv_converter(const gauss_curv_converter &) = default;
    gauss_curv_converter(gauss_curv_converter &&)      = default;
    gauss_curv_converter &operator=(const gauss_curv_converter &) = default;
    gauss_curv_converter &operator=(gauss_curv_converter &&) = default;
    ~gauss_curv_converter() override                         = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;

    double lower_bound;
    double upper_bound;
};

/// Convert range-images to mean curvature images.
/// \sa conversion::depth_to_mean_curvature
class mean_curv_converter : public batch_pinhole_converter {
  public:
    /// \param files,t,intrinsic normal parameters for batch conversion
    /// \param lower_bound,upper_bound clamping parameters. Values below/above
    /// will map to these values.
    mean_curv_converter(const file_patterns &files, depth_type t,
                        const camera_models::pinhole &intrinsic,
                        double lower_bound, double upper_bound)
        : batch_pinhole_converter(files, t, intrinsic)
        , lower_bound{lower_bound}
        , upper_bound{upper_bound} {}
    mean_curv_converter(const mean_curv_converter &) = default;
    mean_curv_converter(mean_curv_converter &&)      = default;
    mean_curv_converter &operator=(const mean_curv_converter &) = default;
    mean_curv_converter &operator=(mean_curv_converter &&) = default;
    ~mean_curv_converter() override                        = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;

    double lower_bound;
    double upper_bound;
};

/// Convert range-images to max-curve images.
/// \sa conversion::depth_to_max_curve
class max_curve_converter : public batch_pinhole_converter {
  public:
    max_curve_converter(const file_patterns &files, depth_type t,
                        const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {}
    max_curve_converter(const max_curve_converter &) = default;
    max_curve_converter(max_curve_converter &&)      = default;
    max_curve_converter &operator=(const max_curve_converter &) = default;
    max_curve_converter &operator=(max_curve_converter &&) = default;
    ~max_curve_converter() override                        = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;
};

/// Convert range-images to flexion images.
/// \sa conversion::depth_to_flexion
class flexion_converter : public batch_pinhole_converter {
  public:
    flexion_converter(const file_patterns &files, depth_type t,
                      const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {}
    flexion_converter(const flexion_converter &) = default;
    flexion_converter(flexion_converter &&)      = default;
    flexion_converter &operator=(const flexion_converter &) = default;
    flexion_converter &operator=(flexion_converter &&) = default;
    ~flexion_converter() override                      = default;

  private:
    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;
};

/// Scale depth images and add a constant value to each pixel.
/// \sa conversion::depth_scaling
class scale_converter : public batch_converter {
  public:
    scale_converter(const file_patterns &files, depth_type t, double scale,
                    double offset)
        : batch_converter(files, t)
        , _scale{scale}
        , _offset{offset} {}
    scale_converter(const scale_converter &) = default;
    scale_converter(scale_converter &&)      = default;
    scale_converter &operator=(const scale_converter &) = default;
    scale_converter &operator=(scale_converter &&) = default;
    ~scale_converter() override                    = default;

  private:
    double _scale  = 1.0;
    double _offset = 0.0;

    [[nodiscard]] bool process_file(cv::Mat depth_image, int idx) const
        noexcept override;
};

/// @}

}}  // namespace sens_loc::apps


#endif /* end of include guard: CONVERTERS_H_HVFGCFVK */
