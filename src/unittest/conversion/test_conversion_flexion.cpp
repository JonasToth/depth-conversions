#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "intrinsic.h"

#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;

TEST_CASE("flexion image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    auto ref_image = io::load_image<ushort>("conversion/flexion-reference.png",
                                            cv::IMREAD_UNCHANGED);
    REQUIRE(ref_image);

    auto laser_double =
        conversion::depth_to_laserscan<double, ushort>(*depth_image, p);

    const auto triple =
        conversion::depth_to_flexion<double, double>(laser_double, p);
    const auto converted = conversion::convert_flexion<double, ushort>(triple);
    cv::imwrite("conversion/test_flexion.png", converted.data());

    REQUIRE(util::average_pixel_error(*ref_image, converted) < 0.5);
}
