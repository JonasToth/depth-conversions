#include <doctest/doctest.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;

TEST_CASE("Loading Images") {
    SUBCASE("Non existing file") {
        std::optional<cv::Mat> file = io::load_image("DoesNotExist");
        REQUIRE(!file);
    }

    SUBCASE("Existing file - JPG") {
        std::optional<cv::Mat> file = io::load_image("io/example-image.jpg");
        REQUIRE(file);
    }

    SUBCASE("Existing file - PNG") {
        std::optional<cv::Mat> file = io::load_image("io/example-image.png");
        REQUIRE(file);
    }

    SUBCASE("Existing file - Unrecognized format") {
        std::optional<cv::Mat> file = io::load_image("io/not_an_image.txt");
        REQUIRE(!file);
    }
}
