#include <doctest/doctest.h>
#include <sens_loc/math/coordinate.h>

using doctest::Approx;
using namespace sens_loc::math;

TEST_CASE("world coordinate") {
    SUBCASE("default construction") {
        coordinate<double, frame::world> p;
        REQUIRE(p.U() == 0.);
        REQUIRE(p.V() == 0.);
        REQUIRE(p.W() == 0.);
    }
    SUBCASE("init Construction") {
        coordinate<double, frame::world> p{10., 2., -2.};
        REQUIRE(p.U() == 10.);
        REQUIRE(p.V() == 2.);
        REQUIRE(p.W() == -2.);
    }
    SUBCASE("assignment") {
        coordinate<double, frame::world> p1{10., 2., -2.};
        coordinate<double, frame::world> p2{11., 2., -2.};
        p1 = p2;
        REQUIRE(p1.U() == 11.);
        REQUIRE(p1.V() == 2.);
        REQUIRE(p1.W() == -2.);
    }
    SUBCASE("norm for real coordinates") {
        coordinate<double, frame::world> p1;
        REQUIRE(p1.norm() == 0.);

        coordinate<double, frame::world> p2(3., 4., 5.);
        REQUIRE(p2.norm() == std::sqrt(50.));
    }
    SUBCASE("dot product") {
        coordinate<double, frame::world> p1, p2;
        REQUIRE(p1.dot(p2) == 0.);

        coordinate<double, frame::world> p3(3., 4., 5.), p4(3., 4., 5.);
        REQUIRE(p3.dot(p4) == 50.);
    }
}

TEST_CASE("pixel coordinate") {
    SUBCASE("default construction") {
        coordinate<int, frame::pixel> p;
        REQUIRE(p.u() == 0);
        REQUIRE(p.v() == 0);
    }
    SUBCASE("init Construction") {
        coordinate<int, frame::pixel> p{100, 200};
        REQUIRE(p.u() == 100);
        REQUIRE(p.v() == 200);
    }
    SUBCASE("assignment") {
        coordinate<int, frame::pixel> p1{100, 200};
        coordinate<int, frame::pixel> p2{112, 22};
        p1 = p2;
        REQUIRE(p1.u() == 112);
        REQUIRE(p1.v() == 22);
    }
}