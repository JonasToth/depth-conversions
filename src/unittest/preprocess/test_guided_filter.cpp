#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/preprocess/filter.h>

using namespace sens_loc;
using namespace preprocess;
using namespace std;

TEST_CASE("filter pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float>(*img, p_float);

    SUBCASE("Small distance --> fast filter") {
        auto res = guided_filter(laser, /*radius=*/3, /*eps=*/10.);
        cv::imwrite("preprocess/test_guided_fast.png",
                    math::convert<ushort>(res).data());
    }

    SUBCASE("Bigger distance --> slow filter") {
        auto laser = conversion::depth_to_laserscan<double>(*img, p_double);
        auto res   = guided_filter(laser, /*radius=*/3, /*eps=*/10.);
        cv::imwrite("preprocess/test_guided_slow.png",
                    math::convert<ushort>(res).data());
    }
}
