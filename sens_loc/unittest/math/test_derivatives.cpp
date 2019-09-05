#include <doctest/doctest.h>
#include <sens_loc/math/derivatives.h>

using namespace sens_loc;
using namespace math;
using doctest::Approx;

TEST_CASE("First derivative central quotient") {
    SUBCASE("Constant function") {
        const double y__1 = 42.;
        const double y_1  = 42.;
        const double dx   = 1.;

        REQUIRE(first_derivative_central(y__1, y_1, dx) == Approx(0.));
    }
    SUBCASE("increasing slope") {
        const double y__1 = 0.;
        const double y_1  = 2.;
        const double dx   = 1.;

        REQUIRE(first_derivative_central(y__1, y_1, dx) == Approx(1.));
    }
    SUBCASE("decreasing slope") {
        const double y__1 = 0.;
        const double y_1  = -2.;
        const double dx   = 1.;

        REQUIRE(first_derivative_central(y__1, y_1, dx) == Approx(-1.));
    }
}

TEST_CASE("second derivative central quotient") {
    SUBCASE("Constant function") {
        const double y__1 = 42.;
        const double y    = 42.;
        const double y_1  = 42.;
        const double dx   = 1.;

        REQUIRE(second_derivative_central(y__1, y, y_1, dx * dx) == Approx(0.));
    }
    SUBCASE("increasing slope") {
        const double y__1 = 0.;
        const double y    = 1.;
        const double y_1  = 2.;
        const double dx   = 1.;

        REQUIRE(second_derivative_central(y__1, y, y_1, dx * dx) == Approx(0.));
    }
    SUBCASE("decreasing slope") {
        const double y__1 = 0.;
        const double y    = -1.;
        const double y_1  = -2.;
        const double dx   = 1.;

        REQUIRE(second_derivative_central(y__1, y, y_1, dx * dx) == Approx(0.));
    }
    SUBCASE("functional increasing slope") {
        const double y__1 = 0.;
        const double y    = 0.5;
        const double y_1  = 2.;
        const double dx   = 1.;

        REQUIRE(second_derivative_central(y__1, y, y_1, dx * dx) == Approx(1.));
    }
    SUBCASE("decreasing slope") {
        const double y__1 = 0.;
        const double y    = -0.5;
        const double y_1  = -2.;
        const double dx   = 1.;

        REQUIRE(second_derivative_central(y__1, y, y_1, dx * dx) ==
                Approx(-1.));
    }
}
