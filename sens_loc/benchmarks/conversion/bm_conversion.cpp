#define NONIUS_RUNNER 1

#include <iostream>
#include <nonius/nonius_single.h++>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <tuple>

using namespace sens_loc;
using namespace conversion;

std::tuple<cv::Mat, cv::Mat, camera_models::pinhole> get_data() {
    const camera_models::pinhole p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };
    const std::optional<cv::Mat> img =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    if (!img)
        throw std::runtime_error{"No Data Found!"};
    const cv::Mat euclid = depth_to_laserscan(*img, p);
    return std::make_tuple(*img, euclid, p);
}

NONIUS_BENCHMARK("Depth2Euclidean", [](nonius::chronometer meter) {
    const auto [depth, _, p] = get_data();
    meter.measure([&] { return depth_to_laserscan(depth, p); });
})
NONIUS_BENCHMARK("Depth2Bearing Diagonal", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    meter.measure(
        [&] { return depth_to_bearing<direction::diagonal>(euclid, p); });
})
NONIUS_BENCHMARK("Depth2Bearing Antidiagonal", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    meter.measure(
        [&] { return depth_to_bearing<direction::antidiagonal>(euclid, p); });
})
NONIUS_BENCHMARK("Depth2Bearing vertical", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    meter.measure(
        [&] { return depth_to_bearing<direction::vertical>(euclid, p); });
})
NONIUS_BENCHMARK("Depth2Bearing horizontal", [](nonius::chronometer meter) {
    const auto [_, euclid, p] = get_data();
    meter.measure(
        [&] { return depth_to_bearing<direction::horizontal>(euclid, p); });
})
