#ifndef BATCH_FILTER_H_OBQ32XCZ
#define BATCH_FILTER_H_OBQ32XCZ

#include "filter_functor.h"

#include <memory>
#include <util/batch_converter.h>
#include <vector>

namespace sens_loc { namespace apps {

class batch_filter : public batch_converter {
  public:
    batch_filter(
        const file_patterns&                                 files,
        const std::vector<std::unique_ptr<abstract_filter>>& operations)
        : batch_converter(files)
        , _operations{operations} {}
    ~batch_filter() override = default;

  private:
    [[nodiscard]] bool process_file(const math::image<double>& depth_image,
                                    int idx) const noexcept override;

    const std::vector<std::unique_ptr<abstract_filter>>& _operations;
};

}}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_FILTER_H_OBQ32XCZ */
