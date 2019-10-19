#include <doctest/doctest.h>
#include <sens_loc/math/coordinate.h>

using doctest::Approx;
using namespace sens_loc::math;

TEST_CASE("world coordinate") {
    SUBCASE("default construction") {
        coordinate<double, coordinate_frame::world> p;
        REQUIRE(p[0] == 0.);
        REQUIRE(p[1] == 0.);
        REQUIRE(p[2] == 0.);
    }
    SUBCASE("init Construction") {
        coordinate<double, coordinate_frame::world> p{10., 2., -2.};
        REQUIRE(p[0] == 10.);
        REQUIRE(p[1] == 2.);
        REQUIRE(p[2] == -2.);
    }
    SUBCASE("assignment") {
        coordinate<double, coordinate_frame::world> p1{10., 2., -2.};
        coordinate<double, coordinate_frame::world> p2{11., 2., -2.};
        p1 = p2;
        REQUIRE(p1[0] == 11.);
        REQUIRE(p1[1] == 2.);
        REQUIRE(p1[2] == -2.);
    }
}

TEST_CASE("pixel coordinate") {
    SUBCASE("default construction") {
        coordinate<int, coordinate_frame::pixel> p;
        REQUIRE(p[0] == 0);
        REQUIRE(p[1] == 0);
    }
    SUBCASE("init Construction") {
        coordinate<int, coordinate_frame::pixel> p{100, 200};
        REQUIRE(p[0] == 100);
        REQUIRE(p[1] == 200);
    }
    SUBCASE("assignment") {
        coordinate<int, coordinate_frame::pixel> p1{100, 200};
        coordinate<int, coordinate_frame::pixel> p2{112, 22};
        p1 = p2;
        REQUIRE(p1[0] == 112);
        REQUIRE(p1[1] == 22);
    }
}
