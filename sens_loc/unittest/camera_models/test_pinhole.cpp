#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>

using namespace sens_loc::camera_models;
using namespace std;

TEST_CASE("Calculate angular resolution") {
    pinhole_parameters p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };
    SUBCASE("angular resolution") {
        REQUIRE(p.x_resolution() == doctest::Approx(0.0015094548));
        REQUIRE(p.y_resolution() == doctest::Approx(0.0018963041));
        REQUIRE(p.x_resolution() < p.y_resolution());
    }
}
