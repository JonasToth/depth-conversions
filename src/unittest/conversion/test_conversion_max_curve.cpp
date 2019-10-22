#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_max_curve.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/angle_conversion.h>
#include <sens_loc/util/correctness_util.h>

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
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    constexpr camera_models::pinhole<double> p = {
        960, 540, 519.226, 479.462, 522.23, 272.737,
    };

    SUBCASE("double accuracy") {
        auto ref_double = io::load_image<ushort>(
            "conversion/max-curve-double.png", cv::IMREAD_UNCHANGED);
        REQUIRE(ref_double);

        const auto laser_double =
            depth_to_laserscan<double, ushort>(*depth_image, p);
        const auto curve_double =
            depth_to_max_curve<double, double>(laser_double, p);
        REQUIRE(!curve_double.data().empty());

        const auto curve_ushort =
            convert_max_curve<double, ushort>(curve_double);
        cv::imwrite("conversion/test_max_curve_double.png",
                    curve_ushort.data());
        REQUIRE(util::average_pixel_error(*ref_double, curve_ushort) < 0.5);
    }
}
