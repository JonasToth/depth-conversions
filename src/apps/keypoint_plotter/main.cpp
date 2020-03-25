#include "batch_plotter.h"

#include <CLI/CLI.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/colored_parse.h>
#include <util/tool_macro.h>
#include <util/version_printer.h>
#include <vector>

/// \defgroup feature-plotter-driver Plot detected keypoints
/// This driver utilizes OpenCV for keypoint and matching plotting.

/// Parallelized driver to batch-process images for keypoint plotting and to
/// plot matches.
/// \ingroup feature-plotter-driver
/// \returns 0 if all images could be processed, 1 if any image fails
MAIN_HEAD("Batch-processing tool to plot keypoints and matches") {
    app.footer("$ keypoint_plotter --input akaze-{:04d}.feature[.gz] \\\n"
               "                   --original-file depth-{}.png      \\\n"
               "                   --output keypoints-depth-{}.png   \\\n"
               "                   --start 0 --end 100 ;\n"
               "> Overwrite the 'source_path' with 'depth-{}.png' to use a\n"
               "> different file then specified in the feature file.\n");

    string feature_file_input_pattern;
    app.add_option("-i,--input", feature_file_input_pattern,
                   "Define file-pattern for the feature files to be plotted")
        ->required();

    optional<string> original_image_input_pattern;
    app.add_option(
        "-f,--original-file", original_image_input_pattern,
        "Optionally add the source path image the keypoints shall be plotted "
        "on. If not provided the feature-file should contain a proper path");

    string output_pattern;
    app.add_option("-o,--output", output_pattern,
                   "Output pattern the final images will be written to. "
                   "NOTE: These are 8bit-RGB images!")
        ->required();

    int start_idx = 0;
    app.add_option("-s,--start", start_idx, "Start index for processing.")
        ->required();
    int end_idx = 0;
    app.add_option("-e,--end", end_idx, "End index for processing.")
        ->required();

    string color = "orange";
    app.add_set("-c,--color", color,
                {"green", "blue", "red", "orange", "purple", "all"},
                "Define the color that shall be used for keypoint plotting",
                /*defaulted=*/true);

    COLORED_APP_PARSE(app, argc, argv);

    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    batch_plotter plotter(feature_file_input_pattern, output_pattern,
                          str_to_color(color), original_image_input_pattern);

    return plotter.process_batch(start_idx, end_idx) ? 0 : 1;
}
MAIN_TAIL
