#include <doctest/doctest.h>
#include <fstream>
#include <sens_loc/io/pose.h>
#include <sstream>

using namespace sens_loc;
using namespace std;
using doctest::Approx;

TEST_CASE("Loading Pose") {
    SUBCASE("Nonexisting File") {
        ifstream unopened_file;
        unopened_file.setstate(std::ios_base::failbit);
        optional<Eigen::Matrix4f> p = io::load_pose(unopened_file);
        REQUIRE(!p);
    }

    SUBCASE("Proper pose with 3x4 matrix format") {
        string intrinsic = "0.998332 -0.001389 -0.057713 -0.013973\n"
                           "0.001450 0.999998 0.001005 -0.006388\n"
                           "0.057711 -0.001087 0.998333 0.102325";
        istringstream fake_file{intrinsic};

        optional<Eigen::Matrix4f> p = io::load_pose(fake_file);
        REQUIRE(p);
        CHECK(p->matrix()(0, 0) == Approx(+0.998332F));
        CHECK(p->matrix()(1, 1) == Approx(+0.999998F));
        CHECK(p->matrix()(2, 2) == Approx(+0.998333F));
        CHECK(p->matrix()(3, 3) == Approx(+1.0F));
        CHECK(p->matrix()(0, 3) == Approx(-0.013973F));
        CHECK(p->matrix()(1, 3) == Approx(-0.006388F));
        CHECK(p->matrix()(2, 3) == Approx(+0.102325F));
        CHECK(p->matrix()(3, 3) == Approx(+1.0F));
    }

    SUBCASE("Not enough rows") {
        string intrinsic = "0.998332 -0.001389 -0.057713 -0.013973\n"
                           "0.057711 -0.001087 0.998333 0.102325";
        istringstream fake_file{intrinsic};
        REQUIRE(!io::load_pose(fake_file));
    }

    SUBCASE("Not enough colums") {
        string intrinsic = "0.998332 -0.057713 -0.013973\n"
                           "0.001450  0.001005 -0.006388\n"
                           "0.057711  0.998333 0.102325";
        istringstream fake_file{intrinsic};
        REQUIRE(!io::load_pose(fake_file));
    }

    SUBCASE("Not rotation and translation only") {
        string intrinsic = "10.00000 -0.001389 -0.057713 -0.013973\n"
                           "0.001450 0.999998 0.001005 -0.006388\n"
                           "0.057711 -0.001087 0.998333 0.102325";
        istringstream fake_file{intrinsic};
        REQUIRE(!io::load_pose(fake_file));
    }
}
