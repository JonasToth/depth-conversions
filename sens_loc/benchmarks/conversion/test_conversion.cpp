#define NONIUS_RUNNER 1
#include <iostream>
#include <nonius/nonius_single.h++>

NONIUS_BENCHMARK("My Name is", [] {
    double sum = 0.;
    for (int i = 0; i < 1000; ++i)
        sum += i;
    nonius::keep_memory(&sum);
});
