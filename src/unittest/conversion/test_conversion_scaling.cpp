#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_scaling.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;

TEST_CASE("scale a depth image") {
    std::optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    std::optional<cv::Mat> ref =
        io::load_image("conversion/scale-up.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref);

    cv::Mat scaled = conversion::depth_scaling<ushort>(*depth_image, 8., 0.);
    cv::imwrite("conversion/test_scale_up.png", scaled);

    REQUIRE(util::average_pixel_error(scaled, *ref) < 0.5);
}


TEST_CASE("Constant offset") {
    std::optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    std::optional<cv::Mat> ref =
        io::load_image("conversion/scale-offset.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref);

    cv::Mat scaled =
        conversion::depth_scaling<ushort>(*depth_image, 1., 10000.);
    cv::imwrite("conversion/test_scale_offset.png", scaled);
    REQUIRE(util::average_pixel_error(scaled, *ref) < 0.5);
}
