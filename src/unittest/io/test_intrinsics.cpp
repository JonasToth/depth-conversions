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
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(unopened_file);
        REQUIRE(!p);
    }
    SUBCASE("Proper intrinsic with 3 lines") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 10.0 250.0\n";
        istringstream fake_file{intrinsic};

        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(p);
        auto params = *p;

        REQUIRE(params.w == 960);
        REQUIRE(params.h == 540);
        REQUIRE(params.fx == 10.0);
        REQUIRE(params.fy == 10.0);
        REQUIRE(params.cx == 500.0);
        REQUIRE(params.cy == 250.0);
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

        REQUIRE(params.w == 960);
        REQUIRE(params.h == 540);
        REQUIRE(params.fx == 10.0);
        REQUIRE(params.fy == 10.0);
        REQUIRE(params.cx == 500.0);
        REQUIRE(params.cy == 250.0);
    }
    SUBCASE("Negative value for dimension") {
        string intrinsic = "-960 540\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for dimension") {
        string intrinsic = "960 0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Negative value for fx") {
        string intrinsic = "960 540\n"
                           "-10.0 0.0 500.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for cx") {
        string intrinsic = "-960 540\n"
                           "10.0 0.0 0.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }

    SUBCASE("Negative value for fy") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 -10.0 250.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Zero value for cy") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 0.0\n"
                           "0.0 10.0 0.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }


    SUBCASE("Line ends to early in dimensions") {
        string intrinsic = "960\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Line ends too early for x-row") {
        string intrinsic = "960 540\n"
                           "10.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
    SUBCASE("Line ends too early in y row") {
        string intrinsic = "960 540\n"
                           "10.0 0.0 500.0\n"
                           "0.0 10.0\n";
        istringstream fake_file{intrinsic};
        optional<pinhole<double>> p = load_pinhole_intrinsic<double>(fake_file);
        REQUIRE(!p);
    }
}
