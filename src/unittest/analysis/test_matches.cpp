#include <doctest/doctest.h>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <sens_loc/analysis/match.h>

using namespace std;
using namespace sens_loc;
using namespace analysis;
using namespace math;
using namespace cv;
using doctest::Approx;

TEST_CASE("matcher does proper gathering") {
    auto queried = imagepoints_t{{15.0F, 20.0F}, {18.0F, 30.0F}, {2.0F, 10.0F}};
    auto trained = imagepoints_t{{10.0F, 20.0F}, {15.0F, 30.0F}};
    auto matches = vector<keypoint_correspondence>{{0, 0}, {1, -1}, {2, 1}};

    auto [query, train] = gather_correspondences(matches, queried, trained);

    REQUIRE(query.size() == 3UL);
    REQUIRE(train.size() == 3UL);

    REQUIRE((query[0] - queried[0]).norm() == Approx(0.0F));
    REQUIRE((query[1] - queried[1]).norm() == Approx(0.0F));
    REQUIRE((query[2] - queried[2]).norm() == Approx(0.0F));

    REQUIRE((train[0] - trained[0]).norm() == Approx(0.0F));
    // The second correspondence has an invalid index for 'train_idx'.
    // Which must result in an invalid point.
    REQUIRE(train[1].u() == Approx(-1.0F));
    REQUIRE(train[1].v() == Approx(-1.0F));
    // The third correspondence is to element 2 in 'trained'
    REQUIRE((train[2] - trained[1]).norm() == Approx(0.0F));
}
