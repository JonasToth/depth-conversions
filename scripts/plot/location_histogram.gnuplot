#!/usr/bin/env gnuplot

plot "location_histo.data" using 1:2:3 with image
pause -1
