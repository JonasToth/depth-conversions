#!/bin/bash

# This script builds all base images in this repository and is used to update
# the docker images for CI.
# It is regularly executed from gitlab-ci scheduler.

set -e
set -o pipefail

SCRIPT_FILE=$(readlink -f "$0")
SCRIPT_PATH=$(dirname "${SCRIPT_FILE}")

# shellcheck source=log_helpers.sh
. "${SCRIPT_PATH}/log_helpers.sh"

print_info "Changing into docker directory"
cd "${SCRIPT_PATH}/../docker/" || (print_error "Could not change into docker/ directory of this repository!"; exit 1)

failure=0
for dir in */; do
    cd "${dir}"
    # remove trailing slash from the variable ${dir}
    img_name="base-image-${dir%/}"
    print_info "Building image in ${dir} with name ${img_name}"

    if ! docker build --pull -t "${img_name}" .; then
        print_warning "Failed to build ${img_name}"
        failure=1
    fi
    # shellcheck disable=SC2103
    cd ..
done

exit "${failure}"
