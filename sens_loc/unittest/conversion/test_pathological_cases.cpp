#include <doctest/doctest.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;
using namespace sens_loc::conversion;
using namespace std;
using doctest::Approx;

TEST_CASE("Real world case found crashing") {
    optional<cv::Mat> depth_image =
        io::load_image("conversion/crashing-file.png", cv::IMREAD_UNCHANGED);

    REQUIRE(depth_image);
    REQUIRE((*depth_image).type() == CV_16U);

    camera_models::pinhole p = {
        .w  = 960,
        .h  = 540,
        .fx = 519.226,
        .fy = 479.462,
        .cx = 522.23,
        .cy = 272.737,
    };

    cv::Mat laser_float = depth_to_laserscan<double, ushort>(*depth_image, p);

    auto vert =
        depth_to_bearing<direction::horizontal, double, double>(laser_float, p);
}
