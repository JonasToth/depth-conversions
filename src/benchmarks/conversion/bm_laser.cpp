#define NONIUS_RUNNER 1
#include <nonius/nonius_single.h++>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include "util.h"

using namespace sens_loc;
using namespace conversion;

NONIUS_BENCHMARK("Depth2Euclidean", [](nonius::chronometer meter) {
    const auto [depth, _, p] = get_data();
    (void) _;
    meter.measure([&] { return depth_to_laserscan(depth, p); });
})

NONIUS_BENCHMARK("Depth2Euclidean parallel", [](nonius::chronometer meter) {
    const auto [depth, euclid, p] = get_data();
    cv::Mat      out              = euclid;
    tf::Executor exe;
    tf::Taskflow flow;
    meter.measure([&] {
        par_depth_to_laserscan(depth, p, out, flow);
        exe.run(flow).wait();
        flow.clear();
    });
})

