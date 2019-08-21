#!/bin/sh

# This script contains utility functions for shell scripts that allow visual
# appealing output.
BOLD=$(tput bold)
RED=$(tput setaf 1)
YELLOW=$(tput setaf 3)
DEFAULT=$(tput sgr0)

print_error() {
    echo "${BOLD}${RED}ERROR:${DEFAULT} $*" >&2
}
print_warning() {
    echo "${BOLD}${YELLOW}WARNING:${DEFAULT} $*" >&2
}
print_info() {
    echo "${BOLD}INFO:${DEFAULT} $*" >&2
}
