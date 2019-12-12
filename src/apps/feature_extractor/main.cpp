#include "batch_extractor.h"

#include <CLI/CLI.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/version_printer.h>
#include <vector>

/// \defgroup feature-extractor-driver feature detection and extraction from
/// images
///
/// This driver utilizes OpenCV for feature detection, extraction and
/// serialization.

/// Parallelized driver to batch-process images for feature detection and
/// extraction.
/// \ingroup feature-extractor-driver
/// \returns 0 if all images could be processed, 1 if any image fails
int main(int argc, char** argv) try {
    using namespace sens_loc;
    using namespace apps;
    using namespace std;

    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    CLI::App app{"Batch-processing tool to extract visual features"};
    app.require_subcommand();  // Expect one or more feature commands
    app.footer("\n\n"
               "An example invocation of the tool is:\n"
               "\n"
               "feature_extractor \\\n"
               "\n");

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    app.add_flag_function("-v,--version", apps::print_version(argv[0]),
                          "Print version and exit");

    std::string arg_input_files;
    app.add_option(
           "-i,--input", arg_input_files,
           "Input pattern for images to filter; e.g. \"flexion-{}.png\"")
        ->required();
    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index of batch, inclusive")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index of batch, inclusive")
        ->required();

    CLI::App* sift_cmd = app.add_subcommand("sift", "Detect SIFT features");
    sift_cmd->footer("\n\n");

    std::string arg_sift_out;
    sift_cmd
        ->add_option("-o,--output", arg_sift_out,
                     "Output file-pattern for sift-features")
        ->required();

    CLI::App* surf_cmd = app.add_subcommand("surf", "Detect SURF features");
    surf_cmd->footer("\n\n");

    std::string arg_surf_out;
    surf_cmd
        ->add_option("-o,--output", arg_surf_out,
                     "Output file-pattern for sift-features")
        ->required();

    CLI11_PARSE(app, argc, argv);

    std::cout << "SIFT to: " << arg_sift_out << "\n"
              << "SURF to: " << arg_surf_out << "\n";

    cv::Ptr<cv::Feature2D> feature = cv::xfeatures2d::SURF::create(400);
    batch_extractor        extractor(feature, arg_input_files, arg_surf_out);

    const bool success = extractor.process_batch(start_idx, end_idx);
    return success ? 0 : 1;
} catch (const std::exception& e) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n"
              << "Message: " << e.what();
    return 1;
} catch (...) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n";
    return 1;
}
