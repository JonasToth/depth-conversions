#include <doctest/doctest.h>
#include <sens_loc/camera_models/concepts.h>
#include <sens_loc/camera_models/equirectangular.h>

using namespace sens_loc::camera_models;
using namespace sens_loc::math;
using namespace std;
using doctest::Approx;

TEST_CASE("concept requirements") {
    static_assert(is_intrinsic_v<equirectangular, double>);
    static_assert(is_intrinsic_v<equirectangular, float>);
}

TEST_CASE("Construction") {
    SUBCASE("default constructor") {
        equirectangular<double> e;
        CHECK(e.w() == 0);
        CHECK(e.h() == 0);
    }
    SUBCASE("width and height constructor") {
        equirectangular<double> e(100, 50);
        CHECK(e.w() == 100);
        CHECK(e.h() == 50);
    }
    SUBCASE("width, height and theta range") {
        equirectangular<double> e(100, 50,
                                  {0.25 * pi<double>, 0.75 * pi<double>});
        (void) e;

        equirectangular<double> e2(100, 50, {0., pi<double>});
        (void) e2;
    }
    SUBCASE("width, height and direct angles") {
        SUBCASE("valid") {
            equirectangular<double> e(100, 50, pi<double> / 4.,
                                      0.5 * pi<double> / 50.);
            CHECK(e.pixel_to_sphere({50, 25}).norm() == Approx(1.0));
        }
        SUBCASE("invalid - must throw") {
            CHECK_THROWS_AS(equirectangular<double>(100, 50, pi<double> / 4.,
                                                    2. * pi<double> / 50.),
                            std::invalid_argument);
        }
    }
}

TEST_CASE("projection to sphere") {
    equirectangular<double> e(1000, 500);

    const double sqrt_2_half = std::sqrt(2.) / 2.;
    SUBCASE("top of sphere") {
        const auto s1 = e.pixel_to_sphere({0, 0});
        CHECK(s1.Xs() == Approx(0.));
        CHECK(s1.Ys() == Approx(0.));
        CHECK(s1.Zs() == Approx(1.));

        const auto s2 = e.pixel_to_sphere({500, 0});
        CHECK(s2.Xs() == Approx(0.));
        CHECK(s2.Ys() == Approx(0.));
        CHECK(s2.Zs() == Approx(1.));

        const auto s3 = e.pixel_to_sphere({999, 0});
        CHECK(s3.Xs() == Approx(0.));
        CHECK(s3.Ys() == Approx(0.));
        CHECK(s3.Zs() == Approx(1.));
    }

    SUBCASE("middle of the sphere") {
        const auto s1 = e.pixel_to_sphere({0, 250});
        CHECK(s1.Xs() == Approx(-1.));
        CHECK(s1.Ys() == Approx(0.));
        CHECK(s1.Zs() == Approx(0.0));

        const auto s2 = e.pixel_to_sphere({250, 250});
        CHECK(s2.Xs() == Approx(0.));
        CHECK(s2.Ys() == Approx(-1.));
        CHECK(s2.Zs() == Approx(0.));

        const auto s3 = e.pixel_to_sphere({500, 250});
        CHECK(s3.Xs() == Approx(1.));
        CHECK(s3.Ys() == Approx(0.));
        CHECK(s3.Zs() == Approx(0.));

        const auto s4 = e.pixel_to_sphere({750, 250});
        CHECK(s4.Xs() == Approx(0.));
        CHECK(s4.Ys() == Approx(1.));
        CHECK(s4.Zs() == Approx(0.));

        const auto s5 = e.pixel_to_sphere({999, 250});
        CHECK(s5.Xs() == Approx(-1.));
        CHECK(s5.Ys() == Approx(0.0062831));
        CHECK(s5.Zs() == Approx(0.));
    }

    SUBCASE("in between") {
        const auto s1 = e.pixel_to_sphere({0, 125});
        CHECK(s1.Xs() == Approx(-sqrt_2_half));
        CHECK(s1.Ys() == Approx(0.));
        CHECK(s1.Zs() == Approx(sqrt_2_half));

        const auto s2 = e.pixel_to_sphere({125, 125});
        CHECK(s2.Xs() == Approx(-0.5));
        CHECK(s2.Ys() == Approx(-0.5));
        CHECK(s2.Zs() == Approx(sqrt_2_half));

        const auto s3 = e.pixel_to_sphere({375, 125});
        CHECK(s3.Xs() == Approx(0.5));
        CHECK(s3.Ys() == Approx(-0.5));
        CHECK(s3.Zs() == Approx(sqrt_2_half));

        const auto s4 = e.pixel_to_sphere({625, 375});
        CHECK(s4.Xs() == Approx(0.5));
        CHECK(s4.Ys() == Approx(0.5));
        CHECK(s4.Zs() == Approx(-sqrt_2_half));

        const auto s5 = e.pixel_to_sphere({875, 375});
        CHECK(s5.Xs() == Approx(-0.5));
        CHECK(s5.Ys() == Approx(0.5));
        CHECK(s5.Zs() == Approx(-sqrt_2_half));
    }

    // These values are not as accurate, because the start of the pixel counts.
    // The conversions therefore don't hit the nadir exactly.
    SUBCASE("bottom of sphere") {
        const auto s1 = e.pixel_to_sphere({0, 499});
        CHECK(s1.Xs() == Approx(-0.00628));
        CHECK(s1.Ys() == Approx(0.));
        CHECK(s1.Zs() == Approx(-1.));

        const auto s2 = e.pixel_to_sphere({500, 499});
        CHECK(s2.Xs() == Approx(0.00628));
        CHECK(s2.Ys() == Approx(0.));
        CHECK(s2.Zs() == Approx(-1.));

        const auto s3 = e.pixel_to_sphere({999, 499});
        CHECK(s3.Xs() == Approx(-0.00628));
        CHECK(s3.Ys() == Approx(0.0000394));
        CHECK(s3.Zs() == Approx(-1.));
    }
}

TEST_CASE("angular resolution") {
}
