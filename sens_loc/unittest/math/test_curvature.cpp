#include <doctest/doctest.h>
#include <sens_loc/math/curvature.h>

using namespace sens_loc;
using namespace math;
using namespace std;
using doctest::Approx;

TEST_CASE("Mean Curvature") {
    SUBCASE("fake") {
        REQUIRE(mean_curvature(0., 0., 0., 0., 0.) == Approx(0.));
    }
}

TEST_CASE("Gaussian Curvature") {
    SUBCASE("fake") {
        REQUIRE(gaussian_curvature(0., 0., 0., 0., 0.) == Approx(0.));
    }
}
