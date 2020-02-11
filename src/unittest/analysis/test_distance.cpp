#include <doctest/doctest.h>
#include <opencv2/core/persistence.hpp>
#include <sens_loc/analysis/distance.h>
#include <sstream>
#include <string>
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

TEST_CASE("write with filestorage") {
    SUBCASE("Write Empty Statistic") {
        analysis::distance d_ana{{}, 4U, "Quantity"};

        cv::FileStorage kp_statistic{
            "keypoint.stat", cv::FileStorage::MEMORY | cv::FileStorage::WRITE |
                                 cv::FileStorage::FORMAT_YAML};
        analysis::write(kp_statistic, "test_empty", d_ana.get_statistic());

        std::string written = kp_statistic.releaseAndGetString();
        REQUIRE(written == "%YAML:1.0\n"
                           "---\n"
                           "test_empty:\n"
                           "   count: 0\n"
                           "   min: 0.\n"
                           "   max: 0.\n"
                           "   median: 0.\n"
                           "   mean: 0.\n"
                           "   variance: 0.\n"
                           "   stddev: 0.\n"
                           "   skewness: 0.\n");
    }

    SUBCASE("Basic Analysis") {
        analysis::distance d_ana{distances, 4U, "Quantity"};
        cv::FileStorage    kp_statistic{
            "keypoint.stat", cv::FileStorage::MEMORY | cv::FileStorage::WRITE |
                                 cv::FileStorage::FORMAT_YAML};
        analysis::write(kp_statistic, "test_basic", d_ana.get_statistic());
        std::string              written = kp_statistic.releaseAndGetString();
        std::istringstream       ss(written);
        std::vector<std::string> lines;
        for (std::string line; std::getline(ss, line);)
            lines.emplace_back(line);

        REQUIRE(lines[0] == "%YAML:1.0");
        REQUIRE(lines[1] == "---");
        REQUIRE(lines[2] == "test_basic:");
        REQUIRE(lines[3] == "   count: 7");
        REQUIRE(lines[4] == "   min: -10.");
        REQUIRE(lines[5] == "   max: 10.");

        // Because boost::accumulators estimates the median the calculation
        // is not exact. For some reason, this differs on machines in the
        // floating point representation.
        // So insteaf of checking the exact value, only the prefix is checked.
        std::string prefix = "   median: 3.33";
        REQUIRE(std::mismatch(prefix.begin(), prefix.end(), lines[6].begin())
                    .first == prefix.end());
        prefix = "   mean: 9.28";
        REQUIRE(std::mismatch(prefix.begin(), prefix.end(), lines[7].begin())
                    .first == prefix.end());
        prefix = "   variance: 3.27";
        REQUIRE(std::mismatch(prefix.begin(), prefix.end(), lines[8].begin())
                    .first == prefix.end());
        prefix = "   stddev: 5.72";
        REQUIRE(std::mismatch(prefix.begin(), prefix.end(), lines[9].begin())
                    .first == prefix.end());
        prefix = "   skewness: -3.93";
        REQUIRE(std::mismatch(prefix.begin(), prefix.end(), lines[10].begin())
                    .first == prefix.end());
    }
}
