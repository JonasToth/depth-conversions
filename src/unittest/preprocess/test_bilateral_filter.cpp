#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/preprocess/filter.h>

using namespace sens_loc;
using namespace preprocess;
using namespace std;

TEST_CASE("bilateral coverage") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    SUBCASE("Small distance --> fast filter") {
        auto laser =
            conversion::depth_to_laserscan<double, ushort>(*img, p_double);
        auto res =
            bilateral_filter(laser, /*sigma_color=*/25., /*proximity=*/5);
        cv::imwrite("preprocess/test_bilateral_fast.png",
                    math::convert<ushort>(res).data());
    }

    SUBCASE("Proximity via sigma value") {
        auto res = bilateral_filter(*img,
                                    /*sigma_color=*/25., /*proximity=*/10.);
        cv::imwrite("preprocess/test_bilateral_sigma.png", res.data());
    }
}
