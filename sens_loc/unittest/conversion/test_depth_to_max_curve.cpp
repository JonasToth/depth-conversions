#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_max_curve.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/angle_conversion.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace std;
using doctest::Approx;

TEST_CASE("testing arkus") {
    SUBCASE("arc cos") {
        REQUIRE(std::acos(0.5) == Approx(math::deg_to_rad(60.)));
        REQUIRE(std::acos(-0.5) == Approx(math::deg_to_rad(120.)));
    }
    SUBCASE("arc sin") {
        REQUIRE(std::asin(0.5) == Approx(math::deg_to_rad(30.)));
        REQUIRE(std::asin(-0.5) == Approx(-math::deg_to_rad(30.)));
    }
}

TEST_CASE("max curve formula") {
    using namespace detail;
    SUBCASE("analytical") {
        const double res = angle_formula(std::sqrt(2.), 1., std::sqrt(2.),
                                         std::cos(math::pi<double> / 4.),
                                         std::cos(math::pi<double> / 4.));
        // math::deg_to_rad(45.));
        REQUIRE(res == Approx(math::pi<double>));
    }
    SUBCASE("really big alpha") {
        const double res =
            angle_formula(1., 1., 1., std::cos(math::pi<double> / 4. - 0.5),
                          std::cos(math::pi<double> / 4. - 0.5));
        REQUIRE(res < math::pi<double>);
        REQUIRE(res > 0.);
    }

    SUBCASE("depth all equal") {
        const double res =
            angle_formula(1., 1., 1., std::cos(0.5), std::cos(0.5));
        REQUIRE(res < math::pi<double>);
        REQUIRE(res > 0.);
    }

    SUBCASE("curvature to inner") {
        const double res =
            angle_formula(10., 1.0, 10., std::cos(0.005), std::cos(0.005));
        REQUIRE(res > math::pi<double>);
        REQUIRE(res < 2. * math::pi<double>);
    }

    SUBCASE("curvature to outer") {
        const double res =
            angle_formula(0.5, 2.0, 0.5, std::cos(0.005), std::cos(0.005));
        INFO(res);
        REQUIRE(res < math::pi<double>);
        REQUIRE(res > 0.);
    }
}

TEST_CASE("depth image to max curve") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    const camera_models::pinhole_parameters p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    SUBCASE("float accuracy") {
        optional<cv::Mat> ref_float = io::load_image(
            "conversion/max-curve-float.png", cv::IMREAD_UNCHANGED);
        REQUIRE(ref_float);

        const cv::Mat laser_float =
            depth_to_laserscan<float, ushort>(*depth_image, p);
        const cv::Mat curve_float = depth_to_max_curve(laser_float, p);
        REQUIRE(!curve_float.empty());

        const cv::Mat curve_ushort =
            convert_max_curve<float, ushort>(curve_float);
        cv::imwrite("conversion/test_max_curve_float.png", curve_ushort);
        REQUIRE(cv::norm(*ref_float - curve_ushort) == Approx(0.0));
    }

    SUBCASE("double accuracy") {
        optional<cv::Mat> ref_double = io::load_image(
            "conversion/max-curve-double.png", cv::IMREAD_UNCHANGED);
        REQUIRE(ref_double);

        const cv::Mat laser_double =
            depth_to_laserscan<double, ushort>(*depth_image, p);
        const cv::Mat curve_double =
            depth_to_max_curve<double, double>(laser_double, p);
        REQUIRE(!curve_double.empty());

        const cv::Mat curve_ushort =
            convert_max_curve<double, ushort>(curve_double);
        cv::imwrite("conversion/test_max_curve_double.png", curve_ushort);
        REQUIRE(cv::norm(*ref_double - curve_ushort) == Approx(0.0));
    }
}
