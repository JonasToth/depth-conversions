#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;
constexpr camera_models::pinhole p = {
    .w  = 960,
    .h  = 540,
    .fx = 519.226,
    .fy = 479.462,
    .cx = 522.23,
    .cy = 272.737,
};

TEST_CASE("flexion image") {
    std::optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);

    std::optional<cv::Mat> ref_image = io::load_image(
        "conversion/flexion-reference.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_image);
    REQUIRE((*ref_image).type() == CV_16U);

    cv::Mat laser_double =
        conversion::depth_to_laserscan<double, ushort>(*depth_image, p);

    const auto triple =
        conversion::depth_to_flexion<double, double>(laser_double, p);
    const auto converted = conversion::convert_flexion<double, ushort>(triple);
    cv::imwrite("conversion/test_flexion.png", converted);

    REQUIRE(util::average_pixel_error(*ref_image, converted) < 0.5);
}
