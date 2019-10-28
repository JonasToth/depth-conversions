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

    auto res = gaussian_blur(laser, /*ksize=*/{5, 5}, /*sigmaX=*/1.);
    cv::imwrite("preprocess/test_gauss.png", math::convert<ushort>(res).data());

    auto flex = conversion::depth_to_flexion(res, p_float);
    cv::imwrite("preprocess/test_gauss_flexion.png",
                conversion::convert_flexion<float, ushort>(flex).data());
}

TEST_CASE("median blur pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    auto res = median_blur(laser, /*ksize=*/5);
    cv::imwrite("preprocess/test_median.png",
                math::convert<ushort>(res).data());

    auto flex = conversion::depth_to_flexion(res, p_float);
    cv::imwrite("preprocess/test_median_flexion.png",
                conversion::convert_flexion<float, ushort>(flex).data());
}

TEST_CASE("blur pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    auto res = blur(laser, /*ksize=*/{5, 5});
    cv::imwrite("preprocess/test_blur.png",
                math::convert<ushort>(res).data());

    auto flex = conversion::depth_to_flexion(res, p_float);
    cv::imwrite("preprocess/test_blur_flexion.png",
                conversion::convert_flexion<float, ushort>(flex).data());
}
