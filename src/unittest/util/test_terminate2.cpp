#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <sens_loc/util/correctness_util.h>

TEST_CASE("Running into unrechable needs to terminate program") {
    sens_loc::util::unreachable("Terminate this test");
}
