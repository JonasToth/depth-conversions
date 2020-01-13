#include "min_dist.h"

#include <CLI/CLI.hpp>
#include <boost/histogram.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/colored_parse.h>
#include <util/tool_macro.h>
#include <util/version_printer.h>
#include <vector>

static cv::NormTypes str_to_norm(std::string_view n) {
    // NOLINT-NEXTLINE(cppcoreguidelines-macro-usage)
#define SWITCH_CV_NORM(NORM_NAME)                                              \
    if (n == #NORM_NAME)                                                       \
        return cv::NormTypes::NORM_##NORM_NAME;
    SWITCH_CV_NORM(L1)
    SWITCH_CV_NORM(L2)
    SWITCH_CV_NORM(L2SQR)
    SWITCH_CV_NORM(HAMMING2)
    SWITCH_CV_NORM(HAMMING)
#undef SWITCH_CV_NORM
    UNREACHABLE("unexpected norm type");  // LCOV_EXCL_LINE
}

MAIN_HEAD("Determine Statistical Characteristica of the Descriptors") {
    // Require exactly one subcommand.
    app.require_subcommand(1);

    string feature_file_input_pattern;
    app.add_option("-i,--input", feature_file_input_pattern,
                   "Define file-pattern for the feature files to be plotted")
        ->required();
    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index for processing.")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index for processing.")
        ->required();

    CLI::App* cmd_min_dist = app.add_subcommand(
        "min-distance", "Calculate the minimum distance of descriptors within "
                        "one image and analyze that.");
    string norm_name = "L2";
    cmd_min_dist->add_set("-n,--norm", norm_name,
                          {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                          "Set the norm that shall be used as distance measure",
                          /*defaulted=*/true);

    COLORED_APP_PARSE(app, argc, argv);

    if (*cmd_min_dist) {
        cv::NormTypes norm_to_use = str_to_norm(norm_name);
        return analyze_min_distance(feature_file_input_pattern, start_idx,
                                    end_idx, norm_to_use);
    }

    UNREACHABLE("Expected to end program with subcommand processing");
}
MAIN_TAIL
