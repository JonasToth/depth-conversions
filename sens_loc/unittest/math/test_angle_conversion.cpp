#include <doctest/doctest.h>
#include <sens_loc/math/angle_conversion.h>

using doctest::Approx;
using namespace sens_loc::math;

TEST_CASE("Angle conversion functions") {
    SUBCASE("Degree to Radian converison") {
        REQUIRE(deg_to_rad(180.) == Approx(pi<double>));
        REQUIRE(deg_to_rad(90.) == Approx(pi<double> / 2.));
        REQUIRE(deg_to_rad(45.) == Approx(pi<double> / 4.));
        REQUIRE(deg_to_rad(360.) == Approx(2. * pi<double>));
        REQUIRE(deg_to_rad(0.) == Approx(0.));
    }
    SUBCASE("Radian to Degree converison") {
        REQUIRE(pi<double> == Approx(deg_to_rad(180.)));
        REQUIRE(pi<double> / 2. == Approx(deg_to_rad(90.)));
        REQUIRE(pi<double> / 4. == Approx(deg_to_rad(45.)));
        REQUIRE(2. * pi<double> == Approx(deg_to_rad(360.)));
        REQUIRE(0. == Approx(deg_to_rad(0.)));
    }
    SUBCASE("Test on the reverse nature of the functions") {
        REQUIRE(180. == Approx(rad_to_deg(deg_to_rad(180.))));
    }
}
