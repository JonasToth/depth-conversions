#!/usr/bin/env sh

if [ $# -ne 2 ]; then
    echo "$0 <input-file> <output-file>"
    echo
    echo "Incorrect usage of the program!"
    exit 1
fi

input="$1"
output="$2"

gnuplot <<EOF
set terminal postscript enhanced eps color
set output "$output"

plot "$input" using 1:2:3 with image
EOF
