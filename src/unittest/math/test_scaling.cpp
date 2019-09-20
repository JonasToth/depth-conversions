#include <doctest/doctest.h>
#include <sens_loc/math/scaling.h>

using namespace sens_loc::math;
using doctest::Approx;

TEST_CASE("actually not scaling") {
    numeric_range<double> unit_range{0., 1.};
    REQUIRE(scale(unit_range, unit_range, 0.5) == Approx(0.5));
}

TEST_CASE("range doubling") {
    numeric_range<double> unit_range{0., 1.};
    numeric_range<double> double_range{0., 2.};

    REQUIRE(scale(unit_range, double_range, 0.5) == Approx(1.0));
    REQUIRE(scale(double_range, unit_range, 1.) == Approx(0.5));
}

TEST_CASE("range doubling") {
    numeric_range<double> unit_range{0., 1.};
    numeric_range<double> double_range{0., 2.};

    REQUIRE(scale(unit_range, double_range, 0.5) == Approx(1.0));
    REQUIRE(scale(double_range, unit_range, 1.) == Approx(0.5));
}

TEST_CASE("signed range to unsigned range") {
    numeric_range<double> range_8bit_signed{-128., 127};
    numeric_range<double> range_16bit_unsigned{0., 65535};

    REQUIRE(scale(range_8bit_signed, range_16bit_unsigned, -128.) ==
            Approx(0.));
    REQUIRE(scale(range_8bit_signed, range_16bit_unsigned, 127.) ==
            Approx(65535.));
    REQUIRE(scale(range_16bit_unsigned, range_8bit_signed, 65535. / 2.) ==
            Approx(-0.5));
}
