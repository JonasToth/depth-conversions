#include <doctest/doctest.h>
#include <sens_loc/util/console.h>
#include <sstream>

TEST_CASE("Writing logging messages to out streams") {
    SUBCASE("Error messages") {
        std::ostringstream ss;

        ss << sens_loc::util::err{};
        REQUIRE(ss.str() == "ERROR: ");

        ss << "My Error Message";
        REQUIRE(ss.str() == "ERROR: My Error Message");
    }
    SUBCASE("Warning messages") {
        std::ostringstream ss;

        ss << sens_loc::util::warn{};
        REQUIRE(ss.str() == "WARN: ");

        ss << "My warning Message";
        REQUIRE(ss.str() == "WARN: My warning Message");
    }
    SUBCASE("Info messages") {
        std::ostringstream ss;

        ss << sens_loc::util::info{};
        REQUIRE(ss.str() == "INFO: ");

        ss << "My Info Message";
        REQUIRE(ss.str() == "INFO: My Info Message");
    }
}
