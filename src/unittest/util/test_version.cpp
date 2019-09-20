#include <algorithm>
#include <doctest/doctest.h>
#include <sens_loc/version.h>

using namespace sens_loc;

TEST_CASE("Create Versionstring") {
    std::string version = get_version();

    // <number>.<number>.<number>
    REQUIRE(version.size() >= 5);
    REQUIRE(std::count_if(std::begin(version), std::end(version),
                          [](char c) { return c == '.'; }) == 2);
}

TEST_CASE("Get Version numbers") {
    REQUIRE(get_major_version() >= 0);
    REQUIRE(get_minor_version() >= 0);
    REQUIRE(get_patch_version() >= 0);
}
