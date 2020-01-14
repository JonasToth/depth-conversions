#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <fmt/core.h>

TEST_CASE("Running into unrechable needs to terminate program") {
    std::string format_string = "depth-{:04d}.png";
    const std::string indexed_string = fmt::format(format_string, 42);

    REQUIRE(indexed_string == "depth-0042.png");
}
