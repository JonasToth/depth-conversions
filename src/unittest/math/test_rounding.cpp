#include <doctest/doctest.h>
#include <sens_loc/math/rounding.h>

using namespace sens_loc::math;
using doctest::Approx;

TEST_CASE("rounding floating point numbers") {
    SUBCASE("after comma") {
        CHECK(roundn(10.056F, 2) == Approx(10.06F));
        CHECK(roundn(10.056F, 1) == Approx(10.1F));
        CHECK(roundn(10.056F, 0) == Approx(10.056F));

        CHECK(roundn(10.056, 2) == Approx(10.06));
        CHECK(roundn(10.056, 1) == Approx(10.1));
        CHECK(roundn(10.056, 0) == Approx(10.056));
    }

    SUBCASE("before the comma") {
        CHECK(roundn(1005.6F, -1) == Approx(1010.0F));
        CHECK(roundn(1905.6F, -3) == Approx(2000.0F));

        CHECK(roundn(1005.6, -1) == Approx(1010.0));
        CHECK(roundn(1905.6, -3) == Approx(2000.0));
    }
}

TEST_CASE("rounding integers") {
    SUBCASE("before comma") {
        CHECK(roundn(10056, 0) == 10056);
        CHECK(roundn(10056UL, 0) == 10056UL);

        CHECK(roundn(10056, -1) == 10060);
        CHECK(roundn(10056UL, -1) == 10060UL);
        CHECK(roundn(10056UL, -5) == 0UL);
    }
}
