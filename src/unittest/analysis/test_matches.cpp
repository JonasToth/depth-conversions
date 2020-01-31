#include <doctest/doctest.h>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <sens_loc/analysis/match.h>

using namespace std;
using namespace sens_loc;
using namespace cv;
using doctest::Approx;

TEST_CASE("matcher does proper gathering") {
    // NOLINTNEXTLINE
    float data0[2][4] = {{1.0F, 2.0F, 3.0F, 0.0F}, {2.0F, 5.0F, 2.0F, 10.0F}};

    Mat  desc0(2, 4, CV_32F, data0);  // NOLINT
    auto kp0 = vector<KeyPoint>{{10.0F, 20.0F, 5.0F}, {15.0F, 30.0F, 5.0F}};

    // NOLINTNEXTLINE
    float data1[3][4] = {{1.2F, 2.0F, 2.8F, 0.0F},
                         {2.8F, 4.9F, 2.2F, 11.0F},
                         {0.0F, 0.0F, 5.0F, 20.0F}};

    Mat  desc1(3, 4, CV_32F, data1);  // NOLINT
    auto kp1 = vector<KeyPoint>{
        {15.0F, 20.0F, 5.0F}, {18.0F, 30.0F, 5.0F}, {2.0F, 10.0F, 5.0F}};

    auto           matcher = BFMatcher::create(NormTypes::NORM_L2, true);
    vector<DMatch> matches;
    matcher->match(desc0, desc1, matches);

    auto [query, train] = analysis::gather_matches(matches, kp0, kp1);

    REQUIRE(query.size() == 2UL);
    REQUIRE(train.size() == 2UL);

    REQUIRE(norm((query[0].pt - kp0[0].pt)) == Approx(0.0F));
    REQUIRE(norm((query[1].pt - kp0[1].pt)) == Approx(0.0F));

    REQUIRE(norm((train[0].pt - kp1[0].pt)) == Approx(0.0F));
    REQUIRE(norm((train[1].pt - kp1[1].pt)) == Approx(0.0F));
}
