#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace std;
using doctest::Approx;

TEST_CASE("convert depth image to laser-scan image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    auto ref_depth_laser_image = io::load_image<ushort>(
        "conversion/data0-depth-laser.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE(ref_depth_laser_image);

    cv::Mat depth_float;
    cv::Mat depth_double;
    depth_image->data().convertTo(depth_float, CV_32F);
    depth_image->data().convertTo(depth_double, CV_64F);

    camera_models::pinhole<float> p_float = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };
    camera_models::pinhole<double> p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    auto laser_float  = depth_to_laserscan(*depth_image, p_float);
    auto laser_double = depth_to_laserscan<double>(*depth_image, p);

    cv::Mat laser_float_16u;
    laser_float.data().convertTo(laser_float_16u, CV_16U);
    cv::Mat laser_double_16u;
    laser_double.data().convertTo(laser_double_16u, CV_16U);

    cv::imwrite("conversion/test_depth_to_laserscan_float.png",
                laser_float_16u);

    REQUIRE(util::average_pixel_error(laser_float_16u,
                                      ref_depth_laser_image->data()) < 0.5);
    REQUIRE(util::average_pixel_error(laser_double_16u, laser_float_16u) < 1.0);
    REQUIRE(util::average_pixel_error(laser_float.data(), depth_float) < 1.0);
}

TEST_CASE("convert depth image to laser-scan image parallel") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    auto ref_depth_laser_image = io::load_image<ushort>(
        "conversion/data0-depth-laser.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE(ref_depth_laser_image);

    cv::Mat depth_float;
    depth_image->data().convertTo(depth_float, CV_32F);

    camera_models::pinhole<float> p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    cv::Mat            laser_out = depth_float;
    math::image<float> laser_img(std::move(laser_out));
    {
        tf::Taskflow flow;
        par_depth_to_laserscan<float>(*depth_image, p, laser_img, flow);
        tf::Executor().run(flow).wait();
    }
    cv::Mat laser_float_16u;
    laser_img.data().convertTo(laser_float_16u, CV_16U);
    REQUIRE(util::average_pixel_error(laser_float_16u,
                                      ref_depth_laser_image->data()) < 5.);
}
