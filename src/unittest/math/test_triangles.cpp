#include <doctest/doctest.h>
#include <sens_loc/math/angle_conversion.h>
#include <sens_loc/math/constants.h>
#include <sens_loc/math/triangles.h>

using namespace sens_loc;
using namespace sens_loc::math;
using doctest::Approx;
using std::cos;
using std::sqrt;

TEST_CASE("bearing angle formula") {
    SUBCASE("regular triangle") {
        const double phi = deg_to_rad(60.);
        REQUIRE(bearing_angle(1.0, 1.0, cos(phi)) == Approx(deg_to_rad(60.)));
    }
    SUBCASE("manually measured triangle sharp") {
        const double phi = deg_to_rad(45.);
        REQUIRE(rad_to_deg(bearing_angle<double>(5.0, 5.0, cos(phi))) ==
                Approx((67.5)));
    }
    SUBCASE("manually measured triangle blunt") {
        const double phi = deg_to_rad(44.);
        REQUIRE(rad_to_deg(bearing_angle<double>(2.4, 6.7, cos(phi))) ==
                Approx(117.468));
    }

    SUBCASE("sharp angle") {
        const double phi           = deg_to_rad(0.05);
        const double sharper_angle = bearing_angle<double>(2.0, 1.0, cos(phi));
        REQUIRE(sharper_angle < deg_to_rad(90.));
        INFO("Sharp angle has " << rad_to_deg(sharper_angle) << " degree");
    }

    SUBCASE("blunt angle") {
        const double phi         = deg_to_rad(0.05);
        const double blunt_angle = bearing_angle<double>(1.0, 2.0, cos(phi));
        REQUIRE(blunt_angle > deg_to_rad(90.));
        INFO("Blunt angle has " << rad_to_deg(blunt_angle) << " degree");
    }

    SUBCASE("rectangular analytical correct") {
        const double phi     = deg_to_rad(60.);
        const double cos_phi = 0.5;
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect    = bearing_angle<double>(d_i, d_j, cos_phi);

        REQUIRE(cos(phi) == Approx(cos_phi));
        REQUIRE(phi == Approx(math::pi<double> / 3.));
        REQUIRE(d_j == Approx(2.));

        REQUIRE(rect == Approx(deg_to_rad(90.)));
        INFO("Rectangular has " << rad_to_deg(rect) << " degree");
    }

    SUBCASE("rectangular with numerical correctness") {
        const double phi     = deg_to_rad(0.05);
        const double cos_phi = cos(phi);
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect    = bearing_angle<double>(d_i, d_j, cos_phi);

        REQUIRE(d_i - d_j * cos_phi == Approx(0.));
        REQUIRE(rect == Approx(deg_to_rad(90.)));
        INFO("Numerical rectangular has " << rad_to_deg(rect) << " degree");
    }

    SUBCASE("another rectangular") {
        const float phi  = deg_to_rad(45.);
        const float d_i  = 1.;
        const float d_j  = sqrt(2.);
        const float rect = bearing_angle<float>(d_i, d_j, cos(phi));

        REQUIRE(rect == Approx(deg_to_rad(90.)));
    }
}

TEST_CASE("comparison to Lin formula" * doctest::should_fail(true)) {
    SUBCASE("regular triangle") {
        const double phi = deg_to_rad(60.);
        REQUIRE(bearing_angle(1.0, 1.0, cos(phi)) ==
                Approx(reference_lin_bearing_angle(1.0, 1.0, cos(phi))));
    }
    SUBCASE("manually measured triangle sharp") {
        const double phi = deg_to_rad(45.);
        CHECK(bearing_angle<double>(5.0, 5.0, cos(phi)) ==
              Approx(reference_lin_bearing_angle<double>(5.0, 5.0, cos(phi))));
    }
    SUBCASE("manually measured triangle blunt") {
        const double phi = deg_to_rad(44.);
        CHECK(bearing_angle<double>(2.4, 6.7, cos(phi)) ==
              Approx(reference_lin_bearing_angle(2.4, 6.7, cos(phi))));
    }

    SUBCASE("sharp angle") {
        const double phi = deg_to_rad(0.05);
        CHECK(bearing_angle<double>(2.0, 1.0, cos(phi)) ==
              Approx(reference_lin_bearing_angle<double>(2.0, 1.0, cos(phi))));
    }

    SUBCASE("blunt angle") {
        const double phi = deg_to_rad(0.05);
        reference_lin_bearing_angle<double>(1.0, 2.0, cos(phi));
        CHECK(bearing_angle<double>(1.0, 2.0, cos(phi)) ==
              Approx(reference_lin_bearing_angle<double>(1.0, 2.0, cos(phi))));
    }

    SUBCASE("rectangular analytical correct") {
        const double phi     = deg_to_rad(60.);
        const double cos_phi = 0.5;
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect =
            reference_lin_bearing_angle<double>(d_i, d_j, cos_phi);

        CHECK(cos(phi) == Approx(cos_phi));
        CHECK(phi == Approx(math::pi<double> / 3.));
        CHECK(d_j == Approx(2.));

        CHECK(rect == Approx(deg_to_rad(90.)));
        INFO("Rectangular has " << rad_to_deg(rect) << " degree");
    }

    SUBCASE("rectangular with numerical correctness") {
        const double phi     = deg_to_rad(0.05);
        const double cos_phi = cos(phi);
        const double d_i     = 1.;
        const double d_j     = d_i / cos_phi;
        const double rect =
            reference_lin_bearing_angle<double>(d_i, d_j, cos_phi);

        CHECK(d_i - d_j * cos_phi == Approx(0.));
        CHECK(rect == Approx(deg_to_rad(90.)));
        INFO("Numerical rectangular has " << rad_to_deg(rect) << " degree");
    }

    SUBCASE("another rectangular") {
        const float phi = deg_to_rad(45.);
        const float d_i = 1.;
        const float d_j = sqrt(2.);
        const float rect =
            reference_lin_bearing_angle<float>(d_i, d_j, cos(phi));

        CHECK(rect == Approx(deg_to_rad(90.)));
    }
}
