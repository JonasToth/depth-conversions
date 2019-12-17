#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Incorrect call!"
    exit 1
fi

exe="$1"
helpers="$2"

. "${helpers}"

print_info "Using \"${exe}\" as driver executable"

if ${exe} ; then
    print_error "Not enough arguments calling not checked"
    exit 1
fi

if ! ${exe} --version ; then
    print_error "Printing the version is required to work"
    exit 1
fi

if ${exe} -i "flexion-{:03d}.png" -s 0 -e 1 ; then
    print_error "Subcommand required"
    exit 1
fi

if ${exe} -i "not-existing-{:03d}.png" -s 0 -e 1 sift -o "foo-{}.png"; then
    print_error "Bad filename needs to fail"
    exit 1
fi

if ! ${exe} -i "flexion-{:03d}.png" -s 0 -e 1 \
     akaze -o "multi-{}.akaze" \
     orb -o "multi-{}.orb" ; then
    print_error "Running multiple detectors with one run failed"
    exit 1
fi
if  [ ! -f multi-0.orb ] || \
    [ ! -f multi-0.akaze ] \
    [ ! -f multi-1.orb ] || \
    [ ! -f multi-1.akaze ] ; then
    print_error "Did not create expected output files."
    exit 1
fi

print_info "Test successful!"
exit 0
