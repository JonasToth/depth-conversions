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
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    SUBCASE("Small distance --> fast filter") {
        auto res = bilateral_filter(laser, /*distance=*/9, /*sigma_color=*/25.,
                                    /*sigma_space=*/25.);
        res = median_blur(res, /*ksize=*/3);
        cv::imwrite("preprocess/test_fast_bilateral.png",
                    math::convert<ushort>(res).data());

        auto flex = conversion::depth_to_flexion(res, p_float);
        cv::imwrite("preprocess/test_fast_bilateral_flexion.png",
                    conversion::convert_flexion<float, ushort>(flex).data());
    }
}

TEST_CASE("filter equirectangular") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/laserscan-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = math::convert<float>(*img);

    SUBCASE("Small distance --> fast filter") {
        math::image<float> res =
            bilateral_filter(laser, /*distance=*/9, /*sigma_color=*/50.,
                             /*sigma_space=*/30.);
        res = median_blur(res, /*ksize=*/5);
        cv::imwrite("preprocess/test_fast_bilateral_laser.png",
                    math::convert<ushort>(res).data());

        auto flex = conversion::depth_to_flexion(res, e_float);
        cv::imwrite("preprocess/test_fast_bilateral_flexion_laser.png",
                    conversion::convert_flexion<float, ushort>(flex).data());
    }
}
