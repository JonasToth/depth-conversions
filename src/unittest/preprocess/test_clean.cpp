#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/image.h>
#include <sens_loc/preprocess/depth_clean.h>

using namespace std;
using namespace sens_loc;
using namespace preprocess;

TEST_CASE("construct filter") {
    depth_cleaner<float> d(7);
}

TEST_CASE("clean images") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);

    SUBCASE("small") {
        depth_cleaner<float> d(3);

        auto res   = d(math::convert<float>(*img));
        auto laser = conversion::depth_to_laserscan<float>(res, p_float);

        const auto triple    = conversion::depth_to_flexion(laser, p_float);
        const auto converted = conversion::convert_flexion<ushort>(triple);
        cv::imwrite("preprocess/test_clean_small.png", converted.data());
    }
    SUBCASE("big") {
        depth_cleaner<float> d(7);

        auto res   = d(math::convert<float>(*img));
        auto laser = conversion::depth_to_laserscan<float>(res, p_float);

        const auto triple    = conversion::depth_to_flexion(laser, p_float);
        const auto converted = conversion::convert_flexion<ushort>(triple);
        cv::imwrite("preprocess/test_clean_big.png", converted.data());
    }
}
