#include <doctest/doctest.h>
#include <sens_loc/analysis/keypoints.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace sens_loc;
using namespace sens_loc::analysis;

const vector<KeyPoint> pts{
    {10.0F, 1.0F, 5.0F}, {5.0F, 5.0F, 5.0F},  {24.0F, 8.0F, 5.0F},
    {12.0F, 4.0F, 5.0F}, {30.0F, 3.0F, 5.0F}, {15.0F, 10.0F, 5.0F},
    {3.0F, 2.0F, 5.0F},
};

TEST_CASE("normal keypoints") {
    SUBCASE("combined configuration functions") {
        keypoints kp_ana;
        kp_ana.configure_image_dimension(35U, 15U);
        kp_ana.configure_size(3, "size distribution");
        kp_ana.configure_response(2, "response distribution");
        kp_ana.configure_distribution(2);
        kp_ana.configure_distribution("width dist", "height dist");

        SUBCASE("disable all analysis") {
            kp_ana.analyze(pts, /*distribution=*/false, /*size=*/false,
                           /*response=*/false);
            CHECK(kp_ana.response().count == 0UL);
            CHECK(kp_ana.response().min == 0.0F);
            CHECK(kp_ana.response().max == 0.0F);
            CHECK(kp_ana.response_histo().size() == 0UL);

            CHECK(kp_ana.size().count == 0UL);
            CHECK(kp_ana.size().min == 0.0F);
            CHECK(kp_ana.size().max == 0.0F);
            CHECK(kp_ana.size_histo().size() == 0UL);

            CHECK(kp_ana.distribution().size() == 0UL);
        }

        SUBCASE("calculate only distribution") {
            kp_ana.enable_response_histo(false);
            kp_ana.enable_size_histo(false);
            kp_ana.configure_distribution(3U, 2U);
            kp_ana.analyze(pts, /*distribution=*/true, /*size=*/false,
                           /*response=*/false);

            CHECK(kp_ana.response().count == 0UL);
            CHECK(kp_ana.response().min == 0.0F);
            CHECK(kp_ana.response().max == 0.0F);
            CHECK(kp_ana.response_histo().size() == 0UL);

            CHECK(kp_ana.size().count == 0UL);
            CHECK(kp_ana.size().min == 0.0F);
            CHECK(kp_ana.size().max == 0.0F);
            CHECK(kp_ana.size_histo().size() == 0UL);

            CHECK(kp_ana.distribution().rank() == 2UL);
            CHECK(kp_ana.distribution().axis(0).size() == 3UL);
            CHECK(kp_ana.distribution().axis(1).size() == 2UL);
            CHECK(kp_ana.distribution().at(0, 0) == 3UL);
        }

        SUBCASE("no distribution and no histos") {
            kp_ana.enable_response_histo(false);
            kp_ana.enable_size_histo(false);
            kp_ana.configure_distribution(3U, 2U);
            kp_ana.analyze(pts, /*distribution=*/false, /*size=*/true,
                           /*response=*/true);

            CHECK(kp_ana.response().count == 7UL);
            CHECK(kp_ana.response().min == 0.0F);
            CHECK(kp_ana.response().max == 0.0F);
            CHECK(kp_ana.response_histo().size() == 0UL);

            CHECK(kp_ana.size().count == 7UL);
            CHECK(kp_ana.size().min == 5.0F);
            CHECK(kp_ana.size().max == 5.0F);
            CHECK(kp_ana.size_histo().size() == 0UL);

            CHECK(kp_ana.distribution().rank() == 2UL);
            CHECK(kp_ana.distribution().axis(0).size() == 0UL);
            CHECK(kp_ana.distribution().axis(1).size() == 0UL);
        }

        SUBCASE("analyse everything") {
            kp_ana.enable_response_histo(true);
            kp_ana.enable_size_histo(true);
            kp_ana.configure_distribution(3U, 2U);
            kp_ana.analyze(pts);

            CHECK(kp_ana.response().count == 7UL);
            CHECK(kp_ana.response().min == 0.0F);
            CHECK(kp_ana.response().max == 0.0F);
            CHECK(kp_ana.response_histo().size() == 2UL);
            CHECK(kp_ana.response_histo().at(1) == 7UL);
            CHECK(kp_ana.response_histo().axis().metadata() ==
                  "response distribution");

            CHECK(kp_ana.size().count == 7UL);
            CHECK(kp_ana.size().min == 5.0F);
            CHECK(kp_ana.size().max == 5.0F);
            CHECK(kp_ana.size_histo().size() == 3UL);
            CHECK(kp_ana.size_histo().at(1) == 7UL);
            CHECK(kp_ana.size_histo().axis().metadata() == "size distribution");

            CHECK(kp_ana.distribution().rank() == 2UL);
            CHECK(kp_ana.distribution().at(0, 0) == 3UL);
            CHECK(kp_ana.distribution().axis(0).size() == 3UL);
            CHECK(kp_ana.distribution().axis(0).metadata() == "width dist");
            CHECK(kp_ana.distribution().axis(1).size() == 2UL);
            CHECK(kp_ana.distribution().axis(1).metadata() == "height dist");
        }
    }
}

TEST_CASE("No keypoints") {
    keypoints kp_ana;
    kp_ana.configure_image_dimension(35U, 15U);
    kp_ana.configure_size(3, "size distribution");
    kp_ana.configure_response(2, "response distribution");
    kp_ana.configure_distribution(2);
    kp_ana.configure_distribution("width dist", "height dist");
    kp_ana.analyze({});

    CHECK(kp_ana.response().count == 0UL);
    CHECK(kp_ana.response().min == 0.0F);
    CHECK(kp_ana.response().max == 0.0F);
    CHECK(kp_ana.size().count == 0UL);
    CHECK(kp_ana.size().min == 0.0F);
    CHECK(kp_ana.size().max == 0.0F);
    CHECK(kp_ana.distribution().rank() == 2UL);
}
