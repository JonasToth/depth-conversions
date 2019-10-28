#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/preprocess/filter.h>

using namespace sens_loc;
using namespace preprocess;
using namespace std;

TEST_CASE("edge-preserving-filter pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    auto res = edge_preserving_filter(laser, /*size=*/3, /*threshold=*/2.);
    cv::imwrite("preprocess/test_edge_preserving.png", res.data());

    auto flex =
        conversion::depth_to_flexion(math::convert<float>(res), p_float);
    cv::imwrite("preprocess/test_edge_preserving_flexion.png",
                conversion::convert_flexion<float, ushort>(flex).data());
}

TEST_CASE("anisotropic pinhole") {
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto laser = conversion::depth_to_laserscan<float, ushort>(*img, p_float);

    auto res = anistropic_diffusion(laser, /*alpha=*/0.05, /*K=*/0.1, 5);
    cv::imwrite("preprocess/test_anisotropic_diffusion.png", res.data());

    auto flex =
        conversion::depth_to_flexion(math::convert<float>(res), p_float);
    cv::imwrite("preprocess/test_anisotropic_diffusion_flexion.png",
                conversion::convert_flexion<float, ushort>(flex).data());
}
