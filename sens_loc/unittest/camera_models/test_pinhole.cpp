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

        // field-of-view in degree for better understandability.
        constexpr double pi = 3.141'592'653'589'793'238'462;
        const double deg_x = (p.x_resolution() * p.w) * (180. / pi);
        REQUIRE(deg_x == doctest::Approx(83.0259736757));
        const double deg_y = (p.y_resolution() * p.h) * (180. / pi);
        REQUIRE(deg_y == doctest::Approx(58.6711201045));
    }
}
