#include "../conversion/intrinsic.h"

#include <doctest/doctest.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/preprocess/filter.h>

using namespace sens_loc;
using namespace preprocess;
using namespace std;

TEST_CASE("gaussian blur pinhole") {
    cv::setNumThreads(0);
    optional<math::image<ushort>> img = io::load_image<ushort>(
        "preprocess/data0-depth.png", cv::IMREAD_UNCHANGED);
    REQUIRE(img);
    auto res = gaussian_blur(*img, /*ksize=*/{3, 3}, /*sigmaX=*/0.5);
    cv::imwrite("preprocess/test_gauss.png", res.data());
}

TEST_CASE("median blur") {
    cv::setNumThreads(0);
    SUBCASE("pinhole") {
        optional<math::image<ushort>> img = io::load_image<ushort>(
            "preprocess/laserscan-depth.png", cv::IMREAD_UNCHANGED);
        REQUIRE(img);
        auto res = median_blur(*img, /*ksize=*/5);
        cv::imwrite("preprocess/test_median.png", res.data());
    }
}
