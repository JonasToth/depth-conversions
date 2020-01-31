#include "sens_loc/camera_models/equirectangular.h"
#include "sens_loc/math/pointcloud.h"

#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>

using namespace sens_loc::camera_models;
using namespace sens_loc::math;
using namespace std;
using namespace cv;
using doctest::Approx;

const auto p = pinhole<double>{/*w=*/1080,    /*h=*/1080,   /*fx=*/2220.0,
                               /*fy=*/2220.0, /*cx=*/540.0, /*cy=*/540.0};

const auto e = equirectangular<double>{1000, 500};
const auto c = pointcloud<double>{
    {+0.0, 0.0, -10.0},
    {-1.0, 2.0, -10.0},
    {+1.0, 2.0, -10.0},
};
const auto i =
    imagepoints<double>{{540.0, 250.0}, {270.0, 270.0}, {810.0, 250.0}};

TEST_CASE("project a pointcloud to image") {
    SUBCASE("pinhole") {
        imagepoints<double> pxs{project_to_image(p, c)};

        CHECK(pxs[0].u() == Approx(540.0));
        CHECK(pxs[0].v() == Approx(540.0));
        CHECK(pxs[1].u() == Approx(762.0));
        CHECK(pxs[1].v() == Approx(96.00));
        CHECK(pxs[2].u() == Approx(318.0));
        CHECK(pxs[2].v() == Approx(96.00));
    }
    SUBCASE("equirectangular") {
        imagepoints<double> pxs{project_to_image(e, c)};

        CHECK(pxs[0].u() == Approx(500.0));
        CHECK(pxs[0].v() == Approx(500.0));
        CHECK(pxs[1].u() == Approx(823.7918088252));
        CHECK(pxs[1].v() == Approx(464.9878259767));
        CHECK(pxs[2].u() == Approx(676.2081911748));
        CHECK(pxs[2].v() == Approx(464.9878259767));
    }
}

TEST_CASE("project image points to sphere") {
    SUBCASE("pinhole") {
        pointcloud<double> pxs{project_to_sphere(p, i)};

        CHECK(pxs[0].X() == Approx(+0.0000000000));
        CHECK(pxs[0].Y() == Approx(-0.1295301311));
        CHECK(pxs[0].Z() == Approx(+0.9915754864));
        CHECK(pxs[1].X() == Approx(-0.1198615732));
        CHECK(pxs[1].Y() == Approx(-0.1198615732));
        CHECK(pxs[1].Z() == Approx(+0.9855284910));
        CHECK(pxs[2].X() == Approx(+0.1197295099));
        CHECK(pxs[2].Y() == Approx(-0.1285983625));
        CHECK(pxs[2].Z() == Approx(+0.9844426370));
    }
    SUBCASE("equirectangular") {
        pointcloud<double> pxs{project_to_sphere(e, i)};

        CHECK(pxs[0].X() == Approx(+0.9685831611));
        CHECK(pxs[0].Y() == Approx(+0.2486898872));
        CHECK(pxs[0].Z() == Approx(+0.0000000000));
        CHECK(pxs[1].X() == Approx(+0.1243449436));
        CHECK(pxs[1].Y() == Approx(-0.9842915806));
        CHECK(pxs[1].Z() == Approx(-0.1253332336));
        CHECK(pxs[2].X() == Approx(-0.3681245527));
        CHECK(pxs[2].Y() == Approx(+0.9297764859));
        CHECK(pxs[2].Z() == Approx(+0.0000000000));
    }
}

TEST_CASE("keypoints to coordinates") {
    std::vector<cv::KeyPoint> kps = coords_to_keypoint(i);
    imagepoints<double>       pts = keypoint_to_coords<double>(kps);

    CHECK(kps.size() == pts.size());
    CHECK(pts.size() == 3UL);
    CHECK(kps[0].pt.x == i[0].u());
    CHECK(kps[0].pt.y == i[0].v());
}

TEST_CASE("coordinates to keypoints") {
    std::vector<cv::KeyPoint> kps = coords_to_keypoint(i);

    CHECK(kps.size() == i.size());
    CHECK(kps.size() == 3UL);
    CHECK(kps[0].pt.x == 540.0F);
    CHECK(kps[0].pt.y == 250.0F);
    CHECK(kps[0].size == 5.0F);
    CHECK(kps[0].response == 0.0F);
    CHECK(kps[1].pt.x == 270.0F);
    CHECK(kps[1].pt.y == 270.0F);
    CHECK(kps[1].size == 5.0F);
    CHECK(kps[1].response == 0.0F);
    CHECK(kps[2].pt.x == 810.0F);
    CHECK(kps[2].pt.y == 250.0F);
    CHECK(kps[2].size == 5.0F);
    CHECK(kps[2].response == 0.0F);
}
