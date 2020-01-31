#include <doctest/doctest.h>
#include <sens_loc/io/pose.h>
#include <sens_loc/math/pointcloud.h>
#include <sstream>

using doctest::Approx;
using namespace sens_loc;
using namespace sens_loc::math;
using namespace std;

TEST_CASE("Pointwise distance for pointcloud from Camera Coordinates") {
    SUBCASE("empty pointcloud") {
        auto diff = pointwise_distance(pointcloud_t{}, pointcloud_t{});
        REQUIRE(diff.size() == 0UL);
    }
    SUBCASE("camera coordinated") {
        pointcloud_t pc0{
            {2.0F, +3.0F, 31.0F},
            {4.0F, +2.0F, 8.0F},
            {9.0F, -2.0F, -10.0F},
        };
        pointcloud_t pc1{
            {-7.0F, 15.0F, 10.0F},
            {+6.0F, 20.0F, 15.0F},
            {-5.0F, 13.0F, 12.0F},
        };
        auto diff = pointwise_distance(pc0, pc1);
        REQUIRE(diff.size() == 3UL);
        REQUIRE(diff[0] ==
                Approx(sqrt(9.0F * 9.0F + 12.0F * 12.0F + 21.0F * 21.0F)));
        REQUIRE(diff[1] ==
                Approx(sqrt(2.0F * 2.0F + 18.0F * 18.0F + 7.0F * 7.0F)));
        REQUIRE(diff[2] ==
                Approx(sqrt(14.0F * 14.0F + 15.0F * 15.0F + 22.0F * 22.0F)));
    }
    SUBCASE("Pointwise distance for pointcloud from Image points") {
        imagepoints_t i0{
            {2.0F, 3.0F},
            {4.0F, 2.0F},
            {9.0F, 2.0F},
        };
        imagepoints_t i1{
            {7.0F, 15.0F},
            {6.0F, 20.0F},
            {5.0F, 13.0F},
        };
        auto diff = pointwise_distance(i0, i1);
        REQUIRE(diff.size() == 3UL);
        REQUIRE(diff[0] == Approx(sqrt(5.0F * 5.0F + 12.0F * 12.0F)));
        REQUIRE(diff[1] == Approx(sqrt(2.0F * 2.0F + 18.0F * 18.0F)));
        REQUIRE(diff[2] == Approx(sqrt(4.0F * 4.0F + 11.0F * 11.0F)));
    }
}


TEST_CASE("relative pose calculation") {
    SUBCASE("diff between identity pose") {
        pose_t i = pose_t::Identity(4, 4);
        pose_t r = relative_pose(i, i);
        REQUIRE(r == pose_t::Identity(4, 4));
    }
    SUBCASE("diff between identity pose and some pose") {
        pose_t p = pose_t::Identity(4, 4);
        p(0, 3)  = +24.0F;  // X-Translation
        p(1, 3)  = -10.0F;  // Y-Translation
        p(2, 3)  = -40.0F;  // Z-Translation
        pose_t r = relative_pose(pose_t::Identity(4, 4), p);
        REQUIRE(r == p);
    }
    SUBCASE("diff between pose and identity pose") {
        pose_t p = pose_t::Identity(4, 4);
        p(0, 3)  = +24.0F;  // X-Translation
        p(1, 3)  = -10.0F;  // Y-Translation
        p(2, 3)  = -40.0F;  // Z-Translation
        pose_t r = relative_pose(p, pose_t::Identity(4, 4));
        REQUIRE(r == p.inverse());
    }
    SUBCASE("general pose to general pose") {
        string s0 = "0.956278 0.021462 -0.291671 -2.260386\n"
                    "-0.021156 0.999767 0.004202 -0.605157\n"
                    "0.291694 0.002153 0.956509 10.008457";
        string s1 = "-0.756851 0.066924 -0.650151 -10.377824\n"
                    "-0.011296 0.993256 0.115392 -0.279230\n"
                    "0.653489 0.094679 -0.750991 8.787344";
        istringstream is0{s0};
        istringstream is1{s1};
        auto          p0 = io::load_pose(is0);
        auto          p1 = io::load_pose(is1);
        pose_t        r  = relative_pose(*p0, *p1);
        REQUIRE((*p0 * r - *p1).norm() == Approx(0.0F));
    }
}

TEST_CASE("pointcloud pose transformation") {
    pointcloud_t origin{{0.0F, 0.0F, 0.0F}};

    SUBCASE("transform with zero-pose") {
        auto t = pose_t::Identity(4, 4) * origin;

        REQUIRE(t.size() == 1UL);
        REQUIRE((t[0] - origin[0]).norm() == Approx(0.0F));
    }

    SUBCASE("transform with translation") {
        pose_t p = pose_t::Identity(4, 4);
        p(0, 3)  = 5.0F;
        p(1, 3)  = 10.0F;
        p(2, 3)  = 20.0F;
        auto t   = p * origin;

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(5.0F));
        REQUIRE(t[0].Y() == Approx(10.0F));
        REQUIRE(t[0].Z() == Approx(20.0F));
    }
    SUBCASE("rotation only x to y") {
        pose_t p = pose_t::Identity(4, 4);
        p(0, 0)  = 0.0F;
        p(1, 0)  = 1.0F;
        auto t   = p * pointcloud_t{{1.0F, 0.0F, 0.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(0.0F));
        REQUIRE(t[0].Y() == Approx(1.0F));
        REQUIRE(t[0].Z() == Approx(0.0F));
    }
    SUBCASE("rotation only x to z") {
        pose_t p = pose_t::Zero(4, 4);
        p(2, 0)  = 1.0F;
        auto t   = p * pointcloud_t{{1.0F, 0.0F, 0.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(0.0F));
        REQUIRE(t[0].Y() == Approx(0.0F));
        REQUIRE(t[0].Z() == Approx(1.0F));
    }
    SUBCASE("rotation only y to z") {
        pose_t p = pose_t::Zero(4, 4);
        p(2, 1)  = 1.0F;
        auto t   = p * pointcloud_t{{0.0F, 1.0F, 0.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(0.0F));
        REQUIRE(t[0].Y() == Approx(0.0F));
        REQUIRE(t[0].Z() == Approx(1.0F));
    }
    SUBCASE("rotation only y to x") {
        pose_t p = pose_t::Zero(4, 4);
        p(0, 1)  = 1.0F;
        auto t   = p * pointcloud_t{{0.0F, 1.0F, 0.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(1.0F));
        REQUIRE(t[0].Y() == Approx(0.0F));
        REQUIRE(t[0].Z() == Approx(0.0F));
    }
    SUBCASE("rotation only z to x") {
        pose_t p = pose_t::Zero(4, 4);
        p(0, 2)  = 1.0F;
        auto t   = p * pointcloud_t{{0.0F, 0.0F, 1.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(1.0F));
        REQUIRE(t[0].Y() == Approx(0.0F));
        REQUIRE(t[0].Z() == Approx(0.0F));
    }
    SUBCASE("rotate and translate") {
        pose_t p = pose_t::Zero(4, 4);
        p(0, 2)  = 1.0F;
        p(0, 3)  = 5.0F;
        auto t   = p * pointcloud_t{{0.0F, 0.0F, 1.0F}};

        REQUIRE(t.size() == 1UL);
        REQUIRE(t[0].X() == Approx(6.0F));
        REQUIRE(t[0].Y() == Approx(0.0F));
        REQUIRE(t[0].Z() == Approx(0.0F));
    }
}
