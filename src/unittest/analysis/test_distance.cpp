#include <doctest/doctest.h>
#include <sens_loc/analysis/distance.h>
#include <vector>

using namespace std;
using namespace sens_loc;

const vector<float> distances{10.0F, 0.0F, 2.0F, 5.0F, -10.F, -2.0F, 1.5F};

TEST_CASE("normal analysis") {
    SUBCASE("Default construction and then analysis") {
        analysis::distance d_ana;
        SUBCASE("with histogram") {
            d_ana.analyze(distances);
            CHECK(d_ana.count() == distances.size());
            CHECK(d_ana.min() == -10.0F);
            CHECK(d_ana.max() == 10.0F);
            // The median is estimated through running quantiles or something
            // like this.
            // This is not so accurate with a small sample size.
            CHECK(d_ana.median() == doctest::Approx(0.3333333F));
            CHECK(d_ana.mean() ==
                  doctest::Approx((5.0F + 1.5F) / distances.size()));
            CHECK(d_ana.variance() != 0.0F);
            CHECK(d_ana.stddev() * d_ana.stddev() ==
                  doctest::Approx(d_ana.variance()));
            CHECK(d_ana.skewness() != 0.0F);

            CHECK(d_ana.histogram().size() == 50UL);
            CHECK(d_ana.histogram().at(0) == 1UL);
            CHECK(d_ana.histogram().axis().metadata() == "distance");
        }

        SUBCASE("without histogram") {
            d_ana.analyze(distances, /*histo=*/false);
            CHECK(d_ana.count() == distances.size());
            CHECK(d_ana.min() == -10.0F);
            CHECK(d_ana.max() == 10.0F);
            // The median is estimated through running quantiles or something
            // like this.
            // This is not so accurate with a small sample size.
            CHECK(d_ana.median() == doctest::Approx(0.3333333F));
            CHECK(d_ana.mean() ==
                  doctest::Approx((5.0F + 1.5F) / distances.size()));
            CHECK(d_ana.variance() != 0.0F);
            CHECK(d_ana.stddev() * d_ana.stddev() ==
                  doctest::Approx(d_ana.variance()));
            CHECK(d_ana.skewness() != 0.0F);

            CHECK(d_ana.histogram().size() == 0UL);
        }
    }

    SUBCASE("Analyze through constructor") {
        analysis::distance d_ana{distances, 4U, "Quantity"};

        CHECK(d_ana.count() == distances.size());
        CHECK(d_ana.min() == -10.0F);
        CHECK(d_ana.max() == 10.0F);
        // The median is estimated through running quantiles or something
        // like this.
        // This is not so accurate with a small sample size.
        CHECK(d_ana.median() == doctest::Approx(0.3333333F));
        CHECK(d_ana.mean() ==
              doctest::Approx((5.0F + 1.5F) / distances.size()));
        CHECK(d_ana.variance() != 0.0F);
        CHECK(d_ana.stddev() * d_ana.stddev() ==
              doctest::Approx(d_ana.variance()));
        CHECK(d_ana.skewness() != 0.0F);

        CHECK(d_ana.histogram().size() == 4UL);
        CHECK(d_ana.histogram().at(0) == 1UL);
        CHECK(d_ana.histogram().axis().metadata() == "Quantity");
    }

    SUBCASE("Empty Data") {
        analysis::distance d_ana{{}, 4U, "Quantity"};
        CHECK(d_ana.count() == 0UL);
    }
}
