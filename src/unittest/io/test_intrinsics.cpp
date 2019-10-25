#include <doctest/doctest.h>
#include <fstream>
#include <sens_loc/io/intrinsics.h>
#include <sstream>

using namespace sens_loc::io;
using namespace sens_loc::camera_models;
using namespace std;

TEST_CASE("Loading Pinhole Intrinsics") {
    SUBCASE("Nonexisting file") {
        ifstream unopened_file;
        unopened_file.setstate(std::ios_base::failbit);
        optional<pinhole<double>> p =
            load_pinhole_intrinsic<double>(unopened_file);
        REQUIRE(!p);
    }
    SUBCASE("Proper intrinsic with 3 lines") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 10.0 250.0";
        istringstream fake_file{intrinsic};

        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(p);
        auto params = *p;

        REQUIRE(params.w() == 960);
        REQUIRE(params.h() == 540);
        REQUIRE(params.fx() == 10.0);
        REQUIRE(params.fy() == 10.0);
        REQUIRE(params.cx() == 500.0);
        REQUIRE(params.cy() == 250.0);
    }
    SUBCASE("Proper intrinsic with more then 2 lines") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 10.0 250.0\n"
                           "0.0 0.0 1.0";
        istringstream fake_file{intrinsic};

        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(p);
        auto params = *p;

        REQUIRE(params.w() == 960);
        REQUIRE(params.h() == 540);
        REQUIRE(params.fx() == 10.0);
        REQUIRE(params.fy() == 10.0);
        REQUIRE(params.cx() == 500.0);
        REQUIRE(params.cy() == 250.0);
    }
    SUBCASE("Negative value for dimension") {
        string                    intrinsic = "-960 540\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for dimension") {
        string                    intrinsic = "960 0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Negative value for fx") {
        string intrinsic = "960 540\n"
                           "-10.0 0.0 500.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for cx") {
        string intrinsic = "-960 540\n"
                           "10.0 0.0 0.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Negative value for fy") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 -10.0 250.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for cy") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 0.0\n"
                           "0.0 10.0 0.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }


    SUBCASE("Line ends to early in dimensions") {
        string                    intrinsic = "960\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Line ends too early for x-row") {
        string intrinsic = "960 540\n"
                           "10.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Line ends too early in y row") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 10.0\n";
        istringstream             fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
}

TEST_CASE("Loading Equirectangular Intrinsics") {
    SUBCASE("Nonexisting file") {
        ifstream unopened_file;
        unopened_file.setstate(std::ios_base::failbit);
        optional<equirectangular<double>> p =
            load_equirectangular_intrinsic<double>(unopened_file);
        REQUIRE(!p);
    }
    SUBCASE("Proper intrinsic with dimension only") {
        string        intrinsic = "3600 1800";
        istringstream fake_file{intrinsic};

        auto p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(p);

        REQUIRE(p->w() == 3600);
        REQUIRE(p->h() == 1800);
    }
    SUBCASE("Proper intrinsic with theta range") {
        string intrinsic = "3600 1800\n"
                           "0.7853981634  2.35619449019";
        istringstream fake_file{intrinsic};

        auto p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(p);

        REQUIRE(p->w() == 3600);
        REQUIRE(p->h() == 1800);
    }
    SUBCASE("Negative value for dimension") {
        string        intrinsic = "-3600 1800";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for dimension") {
        string        intrinsic = "3600 0\n";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Negative value for theta_min") {
        string intrinsic = "3600 1800\n"
                           "-10.0 500.0";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Negative value for theta_max") {
        string intrinsic = "960 540\n"
                           "10.0 -500.0";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for theta_max") {
        string intrinsic = "960 540\n"
                           "0.0 0.0\n";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }


    SUBCASE("theta_min too big") {
        string intrinsic = "3600 1800\n"
                           "10.0 90.0";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("theta_max too big") {
        string intrinsic = "3600 1800\n"
                           "0.0 10.0";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("wrong ordering of theta values") {
        string intrinsic = "3600 1800\n"
                           "1.5 0.7\n";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Line ends to early in dimensions") {
        string        intrinsic = "960\n";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Line ends too early for theta row") {
        string intrinsic = "960 540\n"
                           "0.4\n";
        istringstream fake_file{intrinsic};
        auto          p = load_equirectangular_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
}
