#!/bin/sh

# This script automatically updates a 3rd-party dependency and is a small
# wrapper around 'git subtree'.
SCRIPT_FILE=$(readlink -f "$0")
SCRIPT_PATH=$(dirname "${SCRIPT_FILE}")

check_upstream_exists() {
    git remote -v show | grep --silent "$1"
    return $?
}
check_commit_exists() {
    git cat-file commit "$1" > /dev/null 2>&1
    return $?
}

# shellcheck source=log_helpers.sh
. "${SCRIPT_PATH}/log_helpers.sh"

if [ $# -ne 2 ]; then
    print_error "Wrong number of arguments."
    echo "Usage: ${BOLD}$0 <projectname> <REVISION>${DEFAULT}"
    echo
    echo "This script updates exactly ONE upstream project to a new revision"
    echo "using 'git subtree pull'."
    echo "It expects the conventions documented in ${BOLD}'docs/git_references.md'${DEFAULT}"
    exit 1
fi

cd "${SCRIPT_PATH}/../" || (print_error "Could not change into repository root directory!"; exit 1)
print_info "Changed to repository base directory"

project_name="$1"
upstream_name="upstream-${project_name}"
check_upstream_exists "${upstream_name}"
upstream_exists="$?"

if [ "${upstream_exists}" -ne 0 ]; then
    print_error "Requested project \'${project_name}\' does not exist"
    print_info "Compare to the list of upstream projects in this repository:"
    git remote -v show
    exit 1
fi

commit_revision="$2"
check_commit_exists "${commit_revision}"
commit_exists="$?"

if [ "${commit_exists}" -ne 0 ]; then
    print_error "Requested revision \'${commit_revision}\' does not exist"
    exit 1
fi

print_info "Fetching from upstream repository with remote-name \'${upstream_name}\'"
git fetch "${upstream_name}" || (print_error "Could not fetch from upstream repository"; exit 1)

print_info "Integrating revison via 'git subtree pull'"
git subtree pull \
    --prefix "${project_name}" "${upstream_name}" "${commit_revision}" \
    --squash || \
    (print_error "Could not pull \'${commit_revision}\' from remote \'${upstream_name}\'!"; \
    exit 1)

print_info "Update project ${upstream_name} to ${commit_revision} successfully."
exit 0
