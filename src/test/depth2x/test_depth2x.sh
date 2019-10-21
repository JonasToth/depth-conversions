#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if grep --silent "Precise Pangolin" /etc/os-release ; then
    print_warning "Skipping Tests on old linux - See #8 for more information!"
    exit 0
fi

set -v

if ${exe} ; then
    print_error "Not enough arguments calling not checked"
    exit 1
fi

if ! ${exe} --version ; then
    print_error "Printing the version is required to work"
    exit 1
fi

if ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4 ; then
    print_error "Subcommand is required"
    exit 1
fi

if ${exe} bearing -i "data0-depth.png" -s 0 -e 4; then
    print_error "Subcommand that requires intrinsic, but is not provided needs to fail"
    exit 1
fi

# Subcommand comes quiet late, as common arguments are added to the exe itself,
# not the subcommands.
if ${exe} -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4 bearing; then
    print_error "One output-pattern required. Failure to enforce that"
    exit 1
fi

# Using the subcommand at the beginning of the executable is supported as well
if ${exe} bearing -c "kinect_intrinsic.txt" -i "data0-depth.png" -s 0 -e 4; then
    print_error "One output-pattern required. Failure to enforce that"
    exit 1
fi
print_info "Test successful!"
exit 0
