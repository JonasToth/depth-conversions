#ifndef BATCH_CONVERTER_H_XDIRBPHG
#define BATCH_CONVERTER_H_XDIRBPHG

#include <opencv2/core/mat.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <string_view>

/// Just local helper for batch conversion tasks over a given index range.

namespace sens_loc { namespace apps {

enum class depth_type { orthografic, euclidean };
inline depth_type str_to_depth_type(std::string_view option) {
    if (option == "pinhole-depth")
        return depth_type::orthografic;
    if (option == "pinhole-range")
        return depth_type::euclidean;

    UNREACHABLE("no other options are allowed as input depth type");
}

struct file_patterns {
    std::string input;
    std::string output;
    std::string horizontal;
    std::string vertical;
    std::string diagonal;
    std::string antidiagonal;
};

inline void check_output_exists(const file_patterns &files) {
    if (files.output.empty())
        throw std::invalid_argument{"output pattern required\n"};
}


class batch_converter {
  public:
    batch_converter(const file_patterns &files, depth_type t)
        : _files{files}
        , _input_depth_type{t} {
        if (files.input.empty())
            throw std::invalid_argument{"input pattern is always required!"};
    }

    /// Process the whole batch calling 'process_file' for each index.
    /// Returns 'false' if any of the indices fails.
    bool process_batch(int start, int end) const noexcept;

    virtual ~batch_converter() = default;

  protected:
    const file_patterns &_files;
    depth_type           _input_depth_type;

  private:
    bool process_index(int idx) const noexcept;

    virtual cv::Mat preprocess_depth(cv::Mat depth_image) const noexcept {
        return depth_image;
    }

    /// Method to process exactly on file. This method is expected to have
    /// no sideeffects and is called in parallel.
    virtual bool process_file(cv::Mat depth_image, int idx) const noexcept = 0;
};

class batch_pinhole_converter : public batch_converter {
  public:
    batch_pinhole_converter(const file_patterns &files, depth_type t,
                            const camera_models::pinhole &intrinsic)
        : batch_converter(files, t)
        , intrinsic{intrinsic} {}

  protected:
    const camera_models::pinhole &intrinsic;

  private:
    cv::Mat preprocess_depth(cv::Mat depth_image) const noexcept override;
};

}}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_CONVERTER_H_XDIRBPHG */
