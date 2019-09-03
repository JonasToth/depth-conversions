#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace std;
using doctest::Approx;

TEST_CASE("convert depth image to laser-scan image") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_depth_laser_image = io::load_image(
        "conversion/data0-depth-laser.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE(ref_depth_laser_image);

    cv::Mat depth_float;
    cv::Mat depth_double;
    depth_image->convertTo(depth_float, CV_32F);
    depth_image->convertTo(depth_double, CV_64F);

    camera_models::pinhole p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    cv::Mat laser_float  = depth_to_laserscan(*depth_image, p);
    cv::Mat laser_double = depth_to_laserscan<double>(*depth_image, p);

    cv::Mat laser_float_16u;
    laser_float.convertTo(laser_float_16u, CV_16U);
    cv::Mat laser_double_16u;
    laser_float.convertTo(laser_double_16u, CV_16U);

    cv::imwrite("conversion/test_depth_to_laserscan_float.png",
                laser_float_16u);

    REQUIRE(cv::norm(laser_float_16u - *ref_depth_laser_image) == 0.0);
    REQUIRE(cv::norm(laser_double_16u - laser_float_16u) == 0.0);
    REQUIRE(cv::norm(laser_float - depth_float) != 0.0);
}
