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
        auto flex = conversion::depth_to_flexion(res, p_double);
        cv::imwrite("preprocess/test_bilateral_fast_flexion.png",
                    conversion::convert_flexion<ushort>(flex).data());
    }
    SUBCASE("Big distance --> slower filter, better result") {
        auto res =
            bilateral_filter(laser, /*sigma_color=*/25., /*proximity=*/9);
        cv::imwrite("preprocess/test_bilateral_slow.png",
                    math::convert<ushort>(res).data());
        auto flex = conversion::depth_to_flexion(res, p_float);
        cv::imwrite("preprocess/test_bilateral_slow_flexion.png",
                    conversion::convert_flexion<ushort>(flex).data());
    }

    SUBCASE("Proximity via sigma value") {
        auto res =
            bilateral_filter(laser, /*sigma_color=*/20., /*proximity=*/20.);
        cv::imwrite("preprocess/test_bilateral_sigma.png",
                    math::convert<ushort>(res).data());
        auto flex = conversion::depth_to_flexion(res, p_float);
        cv::imwrite("preprocess/test_bilateral_sigma_flexion.png",
                    conversion::convert_flexion<ushort>(flex).data());
    }
}

TEST_CASE("filter equirectangular") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/laserscan-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = math::convert<float>(*img);

    SUBCASE("Small distance --> fast filter") {
        math::image<float> res =
            bilateral_filter(laser, /*sigma_color=*/25., /*proximity=*/5);
        res = median_blur(res, /*ksize=*/5);
        cv::imwrite("preprocess/test_bilateral_fast_laser.png",
                    math::convert<ushort>(res).data());

        auto flex = conversion::depth_to_flexion(res, e_float);
        cv::imwrite("preprocess/test_bilateral_fast_flexion_laser.png",
                    conversion::convert_flexion<ushort>(flex).data());
    }
}
