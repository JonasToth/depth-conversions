#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_triple.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;
constexpr camera_models::pinhole p = {
    .w  = 960,
    .h  = 540,
    .fx = 519.226,
    .fy = 479.462,
    .cx = 522.23,
    .cy = 272.737,
};

TEST_CASE("triple product normalized") {
    std::optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);

    cv::Mat laser_double =
        conversion::depth_to_laserscan<double, ushort>(*depth_image, p);

    const auto triple =
        conversion::depth_to_triple<double, double>(laser_double, p);
    cv::imwrite("conversion/test_triple.png", triple);
}
