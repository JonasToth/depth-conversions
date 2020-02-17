#!/usr/bin/env sh

if [ $# -ne 3 ]; then
    echo "$0 <input-file> <output-file> <title>"
    echo
    echo "Incorrect usage of the program!"
    exit 1
fi

input="$1"
output="$2"
title="$3"

gnuplot <<EOF
set terminal postscript enhanced eps color
set output "$output"

set title "$title" noenhanced

set yrange [200:0]
set xrange [0:]

plot "$input" using 1:2:3 with image notitle
EOF
