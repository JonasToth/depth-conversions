#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace sens_loc::math;
using namespace std;
using doctest::Approx;

TEST_CASE("Acces Prior Pixel") {
    using conversion::detail::pixel;
    SUBCASE("horizontal") {
        pixel<int, direction::horizontal> p;
        const pixel_coord<int>            pp = p({1, 1});
        REQUIRE(pp.u() == 0);
        REQUIRE(pp.v() == 1);
    }
    SUBCASE("horizontal") {
        pixel<int, direction::vertical> p;
        const pixel_coord<int>          pp = p({1, 1});
        REQUIRE(pp.u() == 1);
        REQUIRE(pp.v() == 0);
    }
    SUBCASE("diagonal") {
        pixel<int, direction::diagonal> p;
        const pixel_coord<int>          pp = p({1, 1});
        REQUIRE(pp.u() == 0);
        REQUIRE(pp.v() == 0);
    }
    SUBCASE("antidiagonal") {
        pixel<int, direction::antidiagonal> p;
        const pixel_coord<int>              pp = p({1, 1});
        REQUIRE(pp.u() == 0);
        REQUIRE(pp.v() == 2);
    }
}

TEST_CASE("iteration range") {
    using conversion::detail::pixel_range;
    cv::Mat img(42, 42, CV_8U);
    SUBCASE("horizontal") {
        pixel_range<direction::horizontal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 0);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("vertical") {
        pixel_range<direction::vertical> r(img);
        REQUIRE(r.x_start == 0);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 1);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("diagonal") {
        pixel_range<direction::diagonal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 1);
        REQUIRE(r.y_end == 42);
    }
    SUBCASE("antidiagonal") {
        pixel_range<direction::antidiagonal> r(img);
        REQUIRE(r.x_start == 1);
        REQUIRE(r.x_end == 42);
        REQUIRE(r.y_start == 0);
        REQUIRE(r.y_end == 41);
    }
}

TEST_CASE("Convert depth image to vertical bearing angle image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto laser_double =
        depth_to_laserscan<double, ushort>(*depth_image, p_double);

    auto ref_vert = io::load_image<uchar>("conversion/bearing-vertical.png",
                                          cv::IMREAD_UNCHANGED);
    REQUIRE(ref_vert);

    auto vertical_bearing =
        depth_to_bearing<direction::vertical>(laser_double, p_double);
    auto converted = convert_bearing<double, uchar>(vertical_bearing);
    cv::imwrite("conversion/test_vertical.png", converted.data());

    REQUIRE(util::average_pixel_error(converted, *ref_vert) < 0.5);
}

TEST_CASE("Convert depth image to vertical bearing angle image in parallel") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto laser = depth_to_laserscan<double, ushort>(*depth_image, p_double);

    auto ref_vert = io::load_image<uchar>("conversion/bearing-vertical.png",
                                          cv::IMREAD_UNCHANGED);
    REQUIRE(ref_vert);

    cv::Mat             out(laser.h(), laser.w(), laser.data().type());
    out = 0.;
    math::image<double> out_img(std::move(out));
    {
        tf::Taskflow flow;
        par_depth_to_bearing<direction::vertical>(laser, p_double, out_img,
                                                  flow);
        tf::Executor().run(flow).wait();
    }
    auto converted = convert_bearing<double, uchar>(out_img);
    cv::imwrite("conversion/test_vertical_parallel.png", converted.data());

    REQUIRE(util::average_pixel_error(converted, *ref_vert) < 0.5);
}

TEST_CASE("Convert depth image to diagonal bearing angle image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto laser_double =
        depth_to_laserscan<double, ushort>(*depth_image, p_double);

    auto ref_diag = io::load_image<ushort>("conversion/bearing-diagonal.png",
                                           cv::IMREAD_UNCHANGED);
    REQUIRE(ref_diag);

    auto diagonal_bearing =
        depth_to_bearing<direction::diagonal>(laser_double, p_double);
    auto converted = convert_bearing<double, ushort>(diagonal_bearing);
    cv::imwrite("conversion/test_diagonal.png", converted.data());

    REQUIRE(util::average_pixel_error(converted, *ref_diag) < 1.0);
}

TEST_CASE("Convert depth image to antidiagonal bearing angle image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    auto laser_float = depth_to_laserscan<float, ushort>(*depth_image, p_float);

    auto ref_anti = io::load_image<uchar>("conversion/bearing-antidiagonal.png",
                                          cv::IMREAD_UNCHANGED);
    REQUIRE(ref_anti);

    auto antidiagonal_bearing =
        depth_to_bearing<direction::antidiagonal>(laser_float, p_float);
    auto converted_bearing =
        convert_bearing<float, uchar>(antidiagonal_bearing);
    cv::imwrite("conversion/test_antidiagonal.png", converted_bearing.data());

    REQUIRE(util::average_pixel_error(converted_bearing, *ref_anti) < 0.5);
}

TEST_CASE("Convert depth image to horizontal bearing angle image") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);
    auto laser_float = depth_to_laserscan<float, ushort>(*depth_image, p_float);
    auto ref_hor = io::load_image<ushort>("conversion/bearing-horizontal.png",
                                          cv::IMREAD_UNCHANGED);
    REQUIRE(ref_hor);

    auto hor_bear =
        depth_to_bearing<direction::horizontal>(laser_float, p_float);
    auto converted_flt = convert_bearing(hor_bear);
    cv::imwrite("conversion/test_horizontal.png", converted_flt.data());

    REQUIRE(util::average_pixel_error(converted_flt, *ref_hor) < 1.0);
}

TEST_CASE("convert laserscan to horizontal bearing angle") {
    auto depth_image = io::load_image<ushort>("conversion/laserscan-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    const auto laser_double = math::convert<double>(*depth_image);
    const auto bearing = conversion::depth_to_bearing<direction::horizontal>(
        laser_double, e_double);
    const auto converted = conversion::convert_bearing<double, ushort>(bearing);
    cv::imwrite("conversion/test_horizontal_bearing_laserscan.png",
                converted.data());

    auto ref_image = io::load_image<ushort>(
        "conversion/bearing-horizontal-laserscan-reference.png",
        cv::IMREAD_UNCHANGED);
    REQUIRE(ref_image);

    REQUIRE(util::average_pixel_error(*ref_image, converted) < 0.5);
}
