#!/bin/sh

if [ $# -ne 3 ]; then
    echo "$0 <input-file> <output-file> <title>"
    echo
    echo "Incorrect usage of the program!"
    exit 1
fi

input="$1"
output="$2"
title="$3"

echo "Plotting histogram with title \"$title\""
gnuplot <<EOF
set terminal pdf color
set output "$output"

set title "$title" noenhanced

set mytics
set grid ytics mytics back ls 0, ls 0
set style histogram rowstacked gap 0
set style fill solid 0.2 border lt -1

plot "$input" using 1:2:3 smooth freq with boxes notitle
EOF
