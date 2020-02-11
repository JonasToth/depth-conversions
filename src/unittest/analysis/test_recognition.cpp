#include "sens_loc/math/pointcloud.h"

#include <doctest/doctest.h>
#include <sens_loc/analysis/recognition_performance.h>

using namespace std;
using namespace sens_loc;
using namespace analysis;
using namespace math;
using namespace cv;
using doctest::Approx;

const vector<DMatch> matches;
const float          threshold = 5.0F;
const imagepoints_t  empty_points;
const imagepoints_t  some_points{{15.0F, 20.0F}, {18.0F, 30.0F}, {2.0F, 10.0F}};
const imagepoints_t  bad_points{{-1.0F, -1.0F}, {-1.0F, -1.0F}, {-1.0F, -1.0F}};

TEST_CASE("Pathological case - no query data") {
    SUBCASE("no query points") {
        element_categories ec{empty_points, some_points, matches, threshold};

        REQUIRE(ec.true_positives.empty());
        REQUIRE(ec.false_positives.empty());
        REQUIRE(ec.true_negatives.empty());
        REQUIRE(ec.false_negatives.empty());

        recognition_statistic rs;
        rs.account(ec);

        CHECK(rs.relevant_elements() == 0L);
        CHECK(rs.irrelevant_elements() == 0L);
        CHECK(rs.total_elements() == 0L);
        CHECK(rs.selected_elements() == 0L);

        CHECK(rs.true_positives() == 0L);
        CHECK(rs.false_positives() == 0L);
        CHECK(rs.true_negatives() == 0L);
        CHECK(rs.false_negatives() == 0L);

        CHECK(rs.precision() == 0.0F);
        CHECK(rs.recall() == 0.0F);
        CHECK(rs.fallout() == 0.0F);
        CHECK(rs.sensitivity() == 0.0F);
        CHECK(rs.specificity() == 0.0F);
        CHECK(rs.rand_index() == 0.0F);
        CHECK(rs.youden_index() == Approx(-1.0F));
    }

    SUBCASE("no train points") {
        element_categories ec{some_points, empty_points, matches, threshold};

        REQUIRE(ec.true_positives.empty());
        REQUIRE(ec.false_positives.empty());
        REQUIRE(ec.true_negatives.empty());
        REQUIRE(ec.false_negatives.empty());

        recognition_statistic rs;
        rs.account(ec);

        CHECK(rs.relevant_elements() == 0L);
        CHECK(rs.irrelevant_elements() == 0L);
        CHECK(rs.total_elements() == 0L);
        CHECK(rs.selected_elements() == 0L);

        CHECK(rs.true_positives() == 0L);
        CHECK(rs.false_positives() == 0L);
        CHECK(rs.true_negatives() == 0L);
        CHECK(rs.false_negatives() == 0L);

        CHECK(rs.precision() == 0.0F);
        CHECK(rs.recall() == 0.0F);
        CHECK(rs.fallout() == 0.0F);
        CHECK(rs.sensitivity() == 0.0F);
        CHECK(rs.specificity() == 0.0F);
        CHECK(rs.rand_index() == 0.0F);
        CHECK(rs.youden_index() == Approx(-1.0F));
    }

    SUBCASE("only invalid train points") {
        element_categories ec{some_points, bad_points, matches, threshold};

        REQUIRE(ec.true_positives.empty());
        REQUIRE(ec.false_positives.empty());
        REQUIRE(ec.true_negatives.size() == 3UL);
        REQUIRE(ec.false_negatives.empty());

        recognition_statistic rs;
        rs.account(ec);

        CHECK(rs.relevant_elements() == 0L);
        CHECK(rs.irrelevant_elements() == 3L);
        CHECK(rs.total_elements() == 3L);
        CHECK(rs.selected_elements() == 0L);

        CHECK(rs.true_positives() == 0L);
        CHECK(rs.false_positives() == 0L);
        CHECK(rs.true_negatives() == 3L);
        CHECK(rs.false_negatives() == 0L);

        CHECK(rs.precision() == 0.0F);
        CHECK(rs.recall() == 0.0F);
        CHECK(rs.fallout() == 0.0F);
        CHECK(rs.sensitivity() == 0.0F);
        CHECK(rs.specificity() == 1.0F);  // == rejection-ratio
        CHECK(rs.rand_index() == 1.0F);   // == Accuracy
        CHECK(rs.youden_index() == Approx(0.0F));
    }
}

TEST_CASE("writing result with filestorage") {
    element_categories    ec{some_points, bad_points, matches, threshold};
    recognition_statistic rs;
    rs.account(ec);

    cv::FileStorage recognition{
        "recognition.stat", cv::FileStorage::MEMORY | cv::FileStorage::WRITE |
                                cv::FileStorage::FORMAT_YAML};
    write(recognition, "classification", rs);
    std::string written = recognition.releaseAndGetString();

    REQUIRE(written == "%YAML:1.0\n"
                       "---\n"
                       "classification:\n"
                       "   relevant_elements: 0\n"
                       "   irrelevant_elements: 3\n"
                       "   total_elements: 3\n"
                       "   selected_elements: 0\n"
                       "   true_positives: 0\n"
                       "   false_positives: 0\n"
                       "   true_negatives: 3\n"
                       "   false_negatives: 0\n"
                       "   precision: 0.\n"
                       "   recall: 0.\n"
                       "   fallout: 0.\n"
                       "   sensitivity: 0.\n"
                       "   specificity: 1.\n"
                       "   rand_index: 1.\n"
                       "   youden_index: 0.\n");
}
