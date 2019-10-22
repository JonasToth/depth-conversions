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
    optional<math::image<ushort>> depth_image = io::load_image<ushort>(
        "conversion/crashing-file.png", cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    constexpr camera_models::pinhole<double> p = {
        960, 540, 519.226, 479.462, 522.23, 272.737,
    };

    auto laser = depth_to_laserscan<double, ushort>(*depth_image, p);
    auto vert =
        depth_to_bearing<direction::horizontal, double, double>(laser, p);
}
