#!/bin/bash

# This script executes a normal build and is configurable via environment
# variables. These are set in '.gitlab-ci.yml'.

set -e
set -o pipefail

SCRIPT_FILE=$(readlink -f "$0")
SCRIPT_PATH=$(dirname "${SCRIPT_FILE}")

# shellcheck source=../log_helpers.sh
. "${SCRIPT_PATH}/../log_helpers.sh"

print_info "Build Debug build with coverage instrumentation"
mkdir -p build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWITH_TESTING=ON -DWITH_TEST_COVERAGE=ON -DWITH_BENCHMARK=OFF
ninja

print_info "Measure coverage"
ctest -j4 --output-on-failure .
lcov --quiet --no-external --capture --directory ../src/apps --directory ../src/include --directory ../src/lib --directory src/ --output-file test_coverage.info
lcov --list test_coverage.info

exit 0