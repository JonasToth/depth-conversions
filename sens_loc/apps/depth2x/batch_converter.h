#ifndef BATCH_CONVERTER_H_XDIRBPHG
#define BATCH_CONVERTER_H_XDIRBPHG

#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <string>

/// Just local helper for batch conversion tasks over a given index range.

namespace sens_loc { namespace apps {

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
    batch_converter(const file_patterns &files)
        : _files{files} {
        if (files.input.empty())
            throw std::invalid_argument{"input pattern is always required!"};
    }

    /// Process the whole batch calling 'process_file' for each index.
    /// Returns 'false' if any of the indices fails.
    bool process_batch(int start, int end) const noexcept;

    virtual ~batch_converter() = default;

  protected:
    const file_patterns &_files;

  private:
    bool process_index(int idx) const noexcept;

    /// Method to process exactly on file. This method is expected to have
    /// no sideeffects and is called in parallel.
    virtual bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept = 0;
};

}}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_CONVERTER_H_XDIRBPHG */
