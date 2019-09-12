#include <cmath>
#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/angle_conversion.h>

using namespace sens_loc::camera_models;
using namespace sens_loc::math;
using namespace std;
using doctest::Approx;

TEST_CASE("Calculate angular resolution") {
    pinhole p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    // Angles must be symmetric.
    REQUIRE(p.phi(0, 1, 1, 1) == p.phi(1, 1, 0, 1));
    REQUIRE(p.phi(1, 0, 1, 1) == p.phi(1, 1, 1, 0));
    REQUIRE(p.phi(1, 1, 0, 0) == p.phi(0, 0, 1, 1));
    REQUIRE(p.phi(1, 0, 0, 1) == p.phi(0, 1, 1, 0));

    // Diagonals create a bigger angle.
    REQUIRE(p.phi(0, 0, 1, 1) > p.phi(0, 0, 1, 0));
    REQUIRE(p.phi(0, 0, 1, 1) > p.phi(0, 0, 0, 1));

    // Angular resolution gets finer in outer parts of image.
    REQUIRE(p.phi(0, 0, 1, 0) < p.phi(480, 0, 481, 0));
    REQUIRE(p.phi(900, 0, 901, 0) < p.phi(480, 0, 481, 0));
}

TEST_CASE("Project pixels to sphere") {
    pinhole p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };
    SUBCASE("horizontal") {
        auto [xs0, ys0, zs0] = p.project_to_sphere(1, 1);
        REQUIRE(xs0 * xs0 + ys0 * ys0 + zs0 * zs0 == Approx(1.));

        auto [xs1, ys1, zs1] = p.project_to_sphere(2, 1);
        REQUIRE(xs1 * xs1 + ys1 * ys1 + zs1 * zs1 == Approx(1.));

        const auto cos_phi = xs0 * xs1 + ys0 * ys1 + zs0 * zs1;
        MESSAGE("horiz: cos(phi) == " << cos_phi);
        const auto angle = std::acos(cos_phi);
        MESSAGE("horiz: acos(phi) == " << rad_to_deg(angle) << "°");
        MESSAGE("horiz: acos(phi) == " << angle << "rad");
    }
    SUBCASE("vertical") {
        auto [xs0, ys0, zs0] = p.project_to_sphere(1, 1);
        REQUIRE(xs0 * xs0 + ys0 * ys0 + zs0 * zs0 == Approx(1.));

        auto [xs1, ys1, zs1] = p.project_to_sphere(1, 2);
        REQUIRE(xs1 * xs1 + ys1 * ys1 + zs1 * zs1 == Approx(1.));

        const auto cos_phi = xs0 * xs1 + ys0 * ys1 + zs0 * zs1;
        MESSAGE("vert: cos(phi) == " << cos_phi);
        const auto angle = std::acos(cos_phi);
        MESSAGE("vert: acos(phi) == " << rad_to_deg(angle) << "°");
        MESSAGE("vert: acos(phi) == " << angle << "rad");
    }

    SUBCASE("diagonal") {
        auto [xs0, ys0, zs0] = p.project_to_sphere(1, 1);
        REQUIRE(xs0 * xs0 + ys0 * ys0 + zs0 * zs0 == Approx(1.));

        auto [xs1, ys1, zs1] = p.project_to_sphere(2, 2);
        REQUIRE(xs1 * xs1 + ys1 * ys1 + zs1 * zs1 == Approx(1.));

        const auto cos_phi = xs0 * xs1 + ys0 * ys1 + zs0 * zs1;
        MESSAGE("diag: cos(phi) == " << cos_phi);
        const auto angle = std::acos(cos_phi);
        MESSAGE("diag: acos(phi) == " << rad_to_deg(angle) << "°");
        MESSAGE("diag: acos(phi) == " << angle << "rad");
    }

    SUBCASE("antidiagonal") {
        auto [xs0, ys0, zs0] = p.project_to_sphere(1, 2);
        REQUIRE(xs0 * xs0 + ys0 * ys0 + zs0 * zs0 == Approx(1.));

        auto [xs1, ys1, zs1] = p.project_to_sphere(2, 1);
        REQUIRE(xs1 * xs1 + ys1 * ys1 + zs1 * zs1 == Approx(1.));

        const auto cos_phi = xs0 * xs1 + ys0 * ys1 + zs0 * zs1;
        MESSAGE("antidiag: cos(phi) == " << cos_phi);
        const auto angle = std::acos(cos_phi);
        MESSAGE("antidiag: acos(phi) == " << rad_to_deg(angle) << "°");
        MESSAGE("antidiag: acos(phi) == " << angle << "rad");
    }
}
