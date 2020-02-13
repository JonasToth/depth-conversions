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

set xrange [-2:]
set mytics
set grid ytics mytics back ls 0, ls 0
set style histogram rowstacked gap 0
set style fill solid 0.2 border lt -1

plot "$input" using 1:2 smooth freq with boxes notitle
EOF
