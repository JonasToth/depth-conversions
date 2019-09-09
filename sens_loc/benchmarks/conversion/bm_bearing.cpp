#define NONIUS_RUNNER 1
#include <nonius/nonius_single.h++>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/io/image.h>
#include "util.h"

using namespace sens_loc;
using namespace conversion;

NONIUS_BENCHMARK("Depth2Bearing Diagonal", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    (void) _;
    meter.measure(
        [&] { return depth_to_bearing<direction::diagonal>(euclid, p); });
})

NONIUS_BENCHMARK("Depth2Bearing Parallel Diagonal",
                 [](nonius::chronometer meter) {
                     const auto [_, euclid, p] = get_data();
                     (void) _;
                     cv::Mat      out = euclid;
                     tf::Executor exe;
                     tf::Taskflow flow;
                     meter.measure([&] {
                         par_depth_to_bearing<direction::diagonal>(euclid, p,
                                                                   out, flow);
                         exe.run(flow).wait();
                         flow.clear();
                     });
                 })

