#!/bin/bash

SCRIPT_FILE=$(readlink -f "$0")
SCRIPT_PATH=$(dirname "${SCRIPT_FILE}")

# shellcheck source=../log_helpers.sh
. "${SCRIPT_PATH}/../log_helpers.sh"

cd "${SCRIPT_PATH}/../../"

print_info "Running Doxygen in repository root directory"
doxygen || (print_error "Could not generate Documentation!"; exit 1)
