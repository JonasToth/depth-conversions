#define NONIUS_RUNNER 1
#include "util.h"

#include <nonius/nonius_single.h++>
#include <sens_loc/conversion/depth_to_curvature.h>

using namespace sens_loc;
using namespace conversion;


NONIUS_BENCHMARK("Depth2Curvature Gaussian", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    (void) _;
    auto in   = euclid;
    auto cali = p;
    meter.measure([&] { return depth_to_gaussian_curvature(in, cali); });
})

NONIUS_BENCHMARK("Depth2Curvature Mean", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    (void) _;
    auto in   = euclid;
    auto cali = p;
    meter.measure([&] { return depth_to_mean_curvature(in, cali); });
})
