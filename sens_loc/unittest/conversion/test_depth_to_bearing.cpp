#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/angle_conversion.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace sens_loc::math;
using namespace std;
using doctest::Approx;

TEST_CASE("bearing angle formula") {
    using namespace detail;
    SUBCASE("regular triangle") {
        const double phi = deg_to_rad(60.);
        REQUIRE(ba_formula<double>(1.0, 1.0, std::cos(phi), phi) ==
                Approx(deg_to_rad(60.)));
    }

    SUBCASE("sharp angle") {
        const double phi = deg_to_rad(0.05);
        const double sharper_angle =
            ba_formula<double>(2.0, 1.0, std::cos(phi), phi);
        REQUIRE(sharper_angle < deg_to_rad(90.));
        INFO("Sharp angle has " << rad_to_deg(sharper_angle) << " degree");
    }

    SUBCASE("blunt angle") {
        const double phi = deg_to_rad(0.05);
        const double blunt_angle =
            ba_formula<double>(1.0, 2.0, std::cos(phi), phi);
        REQUIRE(blunt_angle > deg_to_rad(90.));
        INFO("Blunt angle has " << rad_to_deg(blunt_angle) << " degree");
    }

    SUBCASE("rectangular analytical correct") {
        const double phi     = deg_to_rad(60.);
        const double cos_phi = 0.5;
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect    = ba_formula<double>(d_i, d_j, cos_phi, phi);

        REQUIRE(std::cos(phi) == Approx(cos_phi));
        REQUIRE(phi == Approx(math::pi<double> / 3.));
        REQUIRE(d_j == Approx(2.));

        REQUIRE(rect == Approx(deg_to_rad(90.)));
        INFO("Rectangular has " << rad_to_deg(rect) << " degree");
    }

    SUBCASE("rectangular with numerical correctness") {
        const double phi     = deg_to_rad(0.05);
        const double cos_phi = std::cos(phi);
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect    = ba_formula<double>(d_i, d_j, cos_phi, phi);

        REQUIRE(d_i - d_j * cos_phi == Approx(0.));
        REQUIRE(rect == Approx(deg_to_rad(90.)));
        INFO("Numerical rectangular has " << rad_to_deg(rect) << " degree");
    }
}

TEST_CASE("Scaling of bearing angle") {
    using namespace detail;

    SUBCASE("scale double to uchar8") {
        const double min_angle = 0.;
        const double max_angle = math::pi<double>;
        const double mid_angle = math::pi<double> / 2.;

        auto [scale, offset] = scaling_factor<double, uchar>();

        REQUIRE(offset == 0.);
        REQUIRE(gsl::narrow_cast<uchar>(max_angle * scale) == 254);
        REQUIRE(gsl::narrow_cast<uchar>(min_angle * scale) == 0);
        REQUIRE(gsl::narrow_cast<uchar>(mid_angle * scale) == 127);
    }
    SUBCASE("scale double to uchar16") {
        const double min_angle = 0.;
        const double max_angle = math::pi<double>;
        const double mid_angle = math::pi<double> / 2.;

        auto [scale, offset] = scaling_factor<double, ushort>();

        REQUIRE(offset == 0.);
        REQUIRE(gsl::narrow_cast<ushort>(max_angle * scale) == 65'535);
        REQUIRE(gsl::narrow_cast<ushort>(min_angle * scale) == 0);
        REQUIRE(gsl::narrow_cast<ushort>(mid_angle * scale) == 65'535 / 2);
    }
    SUBCASE("scale double to char8") {
        const double min_angle = 0.;
        const double max_angle = math::pi<double>;
        const double mid_angle = math::pi<double> / 2.;

        auto [scale, offset] = scaling_factor<double, schar>();

        REQUIRE(offset == 128.);
        REQUIRE(gsl::narrow_cast<schar>(max_angle * scale + offset) == 127);
        REQUIRE(gsl::narrow_cast<schar>(min_angle * scale + offset) == -128);
        REQUIRE(gsl::narrow_cast<schar>(mid_angle * scale + offset) == -1);
    }
    SUBCASE("scale double to char16") {
        const double min_angle = 0.;
        const double max_angle = math::pi<double>;
        const double mid_angle = math::pi<double> / 2.;

        auto [scale, offset] = scaling_factor<double, short>();

        REQUIRE(offset == 32768.);
        REQUIRE(gsl::narrow_cast<short>(max_angle * scale + offset) == 32'767);
        REQUIRE(gsl::narrow_cast<short>(min_angle * scale + offset) == -32'768);
        REQUIRE(gsl::narrow_cast<short>(mid_angle * scale + offset) == -1);
    }
}

TEST_CASE("Convert depth image to bearing angle image") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/data0-depth.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_hor_float = io::load_image(
        "conversion/bearing-horizontal-float.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_hor_double = io::load_image(
        "conversion/bearing-horizontal-double.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_vert =
        io::load_image("conversion/bearing-vertical.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_diag =
        io::load_image("conversion/bearing-diagonal.png", cv::IMREAD_UNCHANGED);
    optional<cv::Mat> ref_anti = io::load_image(
        "conversion/bearing-antidiagonal.png", cv::IMREAD_UNCHANGED);

    REQUIRE(depth_image);
    REQUIRE(ref_hor_float);
    REQUIRE(ref_hor_double);
    REQUIRE(ref_vert);
    REQUIRE(ref_diag);
    REQUIRE(ref_anti);
    REQUIRE((*depth_image).type() == CV_16U);

    cv::Mat depth_16bit = *depth_image;
    cv::Mat depth_8bit;
    cv::Mat depth_float;
    cv::Mat depth_double;

    depth_16bit.convertTo(depth_8bit, CV_8U, 255.0 / 65536.0);
    cv::imwrite("conversion/data0-depth-8bit.png", depth_8bit);

    camera_models::pinhole_parameters p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };
    // FIXME: Does not work with 8-bit depth image, nor does it work with
    // floats as pixel type.

    auto horizontal_bearing_float =
        depth_to_bearing<bearing_direction::horizontal, float, ushort>(
            depth_16bit, p);
    auto horizontal_bearing_double =
        depth_to_bearing<bearing_direction::horizontal, float, ushort>(
            depth_16bit, p);

    auto vertical_bearing =
        depth_to_bearing<bearing_direction::vertical, double, ushort>(
            depth_16bit, p);

    auto diagonal_bearing =
        depth_to_bearing<bearing_direction::diagonal, double, ushort>(
            depth_16bit, p);

    auto antidiagonal_bearing =
        depth_to_bearing<bearing_direction::antidiagonal, double, ushort>(
            depth_16bit, p);

    cv::imwrite("conversion/test_horizontal_float.png",
                horizontal_bearing_float);
    cv::imwrite("conversion/test_horizontal_double.png",
                horizontal_bearing_double);
    cv::imwrite("conversion/test_vertical.png",
                vertical_bearing);
    cv::imwrite("conversion/test_diagonal.png",
                diagonal_bearing);
    cv::imwrite("conversion/test_antidiagonal.png",
                antidiagonal_bearing);
    REQUIRE(cv::norm(horizontal_bearing_float - *ref_hor_float) == Approx(0.0));
    REQUIRE(cv::norm(horizontal_bearing_double - *ref_hor_double) ==
            Approx(0.0));
    REQUIRE(cv::norm(horizontal_bearing_double - *ref_hor_double) ==
            Approx(0.0));
    REQUIRE(cv::norm(vertical_bearing - *ref_vert) == Approx(0.0));
    REQUIRE(cv::norm(diagonal_bearing - *ref_diag) == Approx(0.0));
    REQUIRE(cv::norm(antidiagonal_bearing - *ref_anti) == Approx(0.0));
}
