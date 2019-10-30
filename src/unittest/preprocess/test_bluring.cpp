#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/preprocess/filter.h>

using namespace sens_loc;
using namespace preprocess;
using namespace std;

TEST_CASE("gaussian blur pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);
    auto res = gaussian_blur(laser, /*ksize=*/{3, 3}, /*sigmaX=*/0.5);
    cv::imwrite("preprocess/test_gauss.png", math::convert<ushort>(res).data());
}

TEST_CASE("median blur") {
    SUBCASE("pinhole") {
        optional<math::image<ushort>> img = io::load_image<ushort>(
            "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
        REQUIRE(img);
        auto laser =
            conversion::depth_to_laserscan<float, ushort>(*img, p_float);

        auto res = median_blur(laser, /*ksize=*/5);
        cv::imwrite("preprocess/test_median.png",
                    math::convert<ushort>(res).data());
    }
    SUBCASE("laserscan") {
        optional<math::image<ushort>> img = io::load_image<ushort>(
            "preprocess/laserscan-depth.png", cv::IMREAD_UNCHANGED);
        REQUIRE(img);

        auto laser = math::convert<float>(*img);
        math::image<float> res = median_blur(laser, /*ksize=*/3);
        cv::imwrite("preprocess/test_median_laserscan.png",
                    math::convert<ushort>(res).data());
    }
}
