#ifndef CONVERTERS_H_HVFGCFVK
#define CONVERTERS_H_HVFGCFVK

#include "batch_converter.h"

#include <gsl/gsl>

namespace sens_loc { namespace apps {

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

    virtual ~bearing_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};

class range_converter : public batch_pinhole_converter {
  public:
    range_converter(const file_patterns &files, depth_type t,
                    const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        check_output_exists(files);
    }

    virtual ~range_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};

class gauss_curv_converter : public batch_pinhole_converter {
  public:
    gauss_curv_converter(const file_patterns &files, depth_type t,
                         const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        check_output_exists(files);
    }
    virtual ~gauss_curv_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};

class mean_curv_converter : public batch_pinhole_converter {
  public:
    mean_curv_converter(const file_patterns &files, depth_type t,
                        const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        check_output_exists(files);
    }
    virtual ~mean_curv_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};

class max_curve_converter : public batch_pinhole_converter {
  public:
    max_curve_converter(const file_patterns &files, depth_type t,
                        const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        check_output_exists(files);
    }
    virtual ~max_curve_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};

class flexion_converter : public batch_pinhole_converter {
  public:
    flexion_converter(const file_patterns &files, depth_type t,
                      const camera_models::pinhole &intrinsic)
        : batch_pinhole_converter(files, t, intrinsic) {
        check_output_exists(files);
    }
    virtual ~flexion_converter() = default;

  private:
    bool process_file(cv::Mat depth_image, int idx) const noexcept override;
};
}}  // namespace sens_loc::apps


#endif /* end of include guard: CONVERTERS_H_HVFGCFVK */
