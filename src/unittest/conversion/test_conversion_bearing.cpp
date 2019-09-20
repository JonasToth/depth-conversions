#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace std;
using doctest::Approx;

constexpr camera_models::pinhole p = {
    .w  = 960,
    .h  = 540,
    .fx = 519.226,
    .fy = 479.462,
    .cx = 522.23,
    .cy = 272.737,
};

TEST_CASE("Acces Prior Pixel") {
    SUBCASE("horizontal") {
        detail::pixel<direction::horizontal> p;
        const auto [up, vp] = p(1, 1);
        REQUIRE(up == 0);
        REQUIRE(vp == 1);
    }
    SUBCASE("horizontal") {
        detail::pixel<direction::vertical> p;
        const auto [up, vp] = p(1, 1);
        REQUIRE(up == 1);
        REQUIRE(vp == 0);
    }
    SUBCASE("diagonal") {
        detail::pixel<direction::diagonal> p;
        const auto [up, vp] = p(1, 1);
        REQUIRE(up == 0);
        REQUIRE(vp == 0);
    }
    SUBCASE("antidiagonal") {
        detail::pixel<direction::antidiagonal> p;
        const auto [up, vp] = p(1, 1);
        REQUIRE(up == 0);
        REQUIRE(vp == 2);
    }
}

TEST_CASE("iteration range") {
    cv::Mat img(42, 42, CV_8U);
    SUBCASE("horizontal") {
        detail::pixel_range<direction::horizontal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 0);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("vertical") {
        detail::pixel_range<direction::vertical> r(img);
        REQUIRE(r.x_start == 0);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 1);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("diagonal") {
        detail::pixel_range<direction::diagonal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 1);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("antidiagonal") {
        detail::pixel_range<direction::antidiagonal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 0);
        REQUIRE(r.y_end == 41);
    }
}

TEST_CASE("Convert depth image to vertical bearing angle image") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    cv::Mat laser_double = depth_to_laserscan<double, ushort>(*depth_image, p);

    optional<cv::Mat> ref_vert =
        io::load_image("conversion/bearing-vertical.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_vert);

    auto vertical_bearing =
        depth_to_bearing<direction::vertical, double, double>(laser_double, p);
    auto converted = convert_bearing<double, uchar>(vertical_bearing);
    cv::imwrite("conversion/test_vertical.png", converted);

    REQUIRE(util::average_pixel_error(converted, *ref_vert) < 0.5);
}

TEST_CASE("Convert depth image to vertical bearing angle image in parallel") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    cv::Mat laser = depth_to_laserscan<double, ushort>(*depth_image, p);

    optional<cv::Mat> ref_vert =
        io::load_image("conversion/bearing-vertical.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_vert);

    cv::Mat out(laser.rows, laser.cols, laser.type());
    {
        tf::Taskflow flow;
        par_depth_to_bearing<direction::vertical, double, double>(laser, p, out,
                                                                  flow);
        tf::Executor().run(flow).wait();
    }
    auto converted = convert_bearing<double, uchar>(out);
    cv::imwrite("conversion/test_vertical_parallel.png", converted);

    REQUIRE(util::average_pixel_error(converted, *ref_vert) < 0.5);
}

TEST_CASE("Convert depth image to diagonal bearing angle image") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    cv::Mat laser_double = depth_to_laserscan<double, ushort>(*depth_image, p);

    optional<cv::Mat> ref_diag =
        io::load_image("conversion/bearing-diagonal.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_diag);

    auto diagonal_bearing =
        depth_to_bearing<direction::diagonal, double, double>(laser_double, p);
    auto converted = convert_bearing<double, ushort>(diagonal_bearing);
    cv::imwrite("conversion/test_diagonal.png", converted);

    REQUIRE(util::average_pixel_error(converted, *ref_diag) < 0.5);
}

TEST_CASE("Convert depth image to antidiagonal bearing angle image") {
    const optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);

    const cv::Mat laser_float =
        depth_to_laserscan<float, ushort>(*depth_image, p);

    const optional<cv::Mat> ref_anti = io::load_image(
        "conversion/bearing-antidiagonal.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_anti);

    const auto antidiagonal_bearing =
        depth_to_bearing<direction::antidiagonal, float, float>(laser_float, p);
    const auto converted_bearing =
        convert_bearing<float, uchar>(antidiagonal_bearing);
    cv::imwrite("conversion/test_antidiagonal.png", converted_bearing);

    REQUIRE(util::average_pixel_error(converted_bearing, *ref_anti) < 0.5);
}

TEST_CASE("Convert depth image to horizontal bearing angle image") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);
    cv::Mat laser_double = depth_to_laserscan<double, ushort>(*depth_image, p);
    cv::Mat laser_float  = depth_to_laserscan<float, ushort>(*depth_image, p);

    optional<cv::Mat> ref_hor_float = io::load_image(
        "conversion/bearing-horizontal-float.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_hor_double = io::load_image(
        "conversion/bearing-horizontal-double.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_diff =
        io::load_image("conversion/horizontal-diff.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_hor_float);
    REQUIRE(ref_hor_double);
    REQUIRE(ref_diff);

    auto horizontal_bearing_float =
        depth_to_bearing<direction::horizontal, float, double>(laser_double, p);
    auto converted_flt = convert_bearing(horizontal_bearing_float);
    cv::imwrite("conversion/test_horizontal_float.png", converted_flt);

    auto horizontal_bearing_double =
        depth_to_bearing<direction::horizontal, double, float>(laser_float, p);
    auto converted_dbl = convert_bearing<double>(horizontal_bearing_double);
    cv::imwrite("conversion/test_horizontal_double.png", converted_dbl);


    REQUIRE(util::average_pixel_error(converted_flt, *ref_hor_float) < 2.0);
    REQUIRE(util::average_pixel_error(converted_dbl, *ref_hor_double) < 2.0);


    cv::Mat diff;
    diff = convert_bearing<double, uchar>(horizontal_bearing_double) -
           convert_bearing<float, uchar>(horizontal_bearing_float);
    cv::imwrite("conversion/test-horizontal-diff.png", diff);
    /// Small Difference, but not zero.
    REQUIRE(util::average_pixel_error(diff, *ref_diff) < 0.5);
}
