#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "intrinsic.h"

#include <doctest/doctest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sens_loc/conversion/depth_to_curvature.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/util/correctness_util.h>

using namespace sens_loc;

TEST_CASE("gaussian curvature") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    auto laser_double =
        conversion::depth_to_laserscan<double, ushort>(*depth_image, p);

    const auto gauss = conversion::depth_to_gaussian_curvature(laser_double, p);
    const auto converted =
        conversion::curvature_to_image(gauss, *depth_image, {-20.}, {20.});
    cv::imwrite("conversion/test_gauss.png", converted.data());

    auto ref_image = io::load_image<ushort>("conversion/gauss-reference.png",
                                            cv::IMREAD_UNCHANGED);
    REQUIRE(ref_image);
    // Note: ASAN builds have a higher error marging vs the normal builds.
    // There is no obvious reason for that and non-asan builds seem to be fine.
    // FIXME: Why?
    REQUIRE(util::average_pixel_error(converted, *ref_image) < 5.);
}

TEST_CASE("mean curvature") {
    auto depth_image = io::load_image<ushort>("conversion/data0-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    auto laser_double =
        conversion::depth_to_laserscan<double, ushort>(*depth_image, p);

    const auto mean = conversion::depth_to_mean_curvature(laser_double, p);

    SUBCASE("clamped conversion") {
        const auto converted =
            conversion::curvature_to_image(mean, *depth_image, {-20.}, {20.});
        cv::imwrite("conversion/test_mean.png", converted.data());

        auto ref_image = io::load_image<ushort>("conversion/mean-reference.png",
                                                cv::IMREAD_UNCHANGED);
        REQUIRE(ref_image);
        // Note: ASAN builds have a higher error marging vs the normal builds.
        // There is no obvious reason for that and non-asan builds seem to be
        // fine.
        // FIXME: Why?
        REQUIRE(util::average_pixel_error(converted, *ref_image) < 5.);
    }

    SUBCASE("unclamped conversion") {
        const auto converted =
            conversion::curvature_to_image(mean, *depth_image);
        cv::imwrite("conversion/test_mean_unclamped.png", converted.data());
    }
}

TEST_CASE("gaussian curvature equirectangular") {
    auto depth_image = io::load_image<ushort>("conversion/laserscan-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    const auto laser_double = math::convert<double>(*depth_image);
    const auto gauss =
        conversion::depth_to_gaussian_curvature(laser_double, e_double);
    const auto converted =
        conversion::curvature_to_image(gauss, *depth_image, {-20.}, {20.});
    cv::imwrite("conversion/test_gauss_laserscan.png", converted.data());

    auto ref_image = io::load_image<ushort>(
        "conversion/gauss-laserscan-reference.png", cv::IMREAD_UNCHANGED);
    REQUIRE(ref_image);
    // Note: ASAN builds have a higher error marging vs the normal builds.
    // There is no obvious reason for that and non-asan builds seem to be fine.
    // FIXME: Why?
    REQUIRE(util::average_pixel_error(converted, *ref_image) < 2.5);
}

TEST_CASE("mean curvature equirectangular") {
    auto depth_image = io::load_image<ushort>("conversion/laserscan-depth.png",
                                              cv::IMREAD_UNCHANGED);
    REQUIRE(depth_image);

    const auto laser_double = math::convert<double>(*depth_image);
    const auto mean =
        conversion::depth_to_mean_curvature(laser_double, e_double);

    SUBCASE("clamped conversion") {
        const auto converted =
            conversion::curvature_to_image(mean, *depth_image, {-20.}, {20.});
        cv::imwrite("conversion/test_mean_laserscan.png", converted.data());

        auto ref_image = io::load_image<ushort>(
            "conversion/mean-laserscan-reference.png", cv::IMREAD_UNCHANGED);
        REQUIRE(ref_image);
        // Note: ASAN builds have a higher error marging vs the normal builds.
        // There is no obvious reason for that and non-asan builds seem to be
        // fine.
        // FIXME: Why?
        REQUIRE(util::average_pixel_error(converted, *ref_image) < 2.5);
    }

    SUBCASE("unclamped conversion") {
        const auto converted =
            conversion::curvature_to_image<ushort>(mean, *depth_image);
        cv::imwrite("conversion/test_mean_laserscan_unclamped.png",
                    converted.data());
    }
}
