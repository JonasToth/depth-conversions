#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "intrinsic.h"

#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_scaling.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;

TEST_CASE("scale a depth image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto ref =
        io::load_image<ushort>("conversion/scale-up.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref);

    auto scaled = conversion::depth_scaling<ushort>(*depth_image, 8., 0.);
    cv::imwrite("conversion/test_scale_up.png", scaled.data());

    REQUIRE(util::average_pixel_error(scaled, *ref) < 0.5);
}


TEST_CASE("Constant offset") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto ref = io::load_image<ushort>("conversion/scale-offset.png",
                                      cv::IMREAD_UNCHANGED);
    REQUIRE(ref);

    auto scaled = conversion::depth_scaling<ushort>(*depth_image, 1., 10000.);
    cv::imwrite("conversion/test_scale_offset.png", scaled.data());
    REQUIRE(util::average_pixel_error(scaled, *ref) < 0.5);
}
