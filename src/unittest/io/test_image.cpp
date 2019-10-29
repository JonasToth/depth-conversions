#include <doctest/doctest.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;

TEST_CASE("Loading Images") {
    SUBCASE("Non existing file") {
        std::optional<math::image<ushort>> file =
            io::load_image<ushort>("DoesNotExist");
        REQUIRE(!file);
    }

    SUBCASE("Existing file - PNG") {
        std::optional<math::image<uchar>> file =
            io::load_image<uchar>("io/example-image.png", cv::IMREAD_UNCHANGED);
        REQUIRE(file);
    }
}
