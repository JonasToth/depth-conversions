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

namespace {

struct SURFArgs {
    SURFArgs(CLI::App* cmd) {
        cmd->add_option("-o,--output", out_path,
                        "Output file-pattern for sift-features")
            ->required();
        cmd->add_option("-t,--threshold", hessian_threshold,
                        "Control the hessian threshold value, between 300-500 "
                        "might be good",
                        /*defaulted=*/true)
            ->check(CLI::Range(0, 1500));
        cmd->add_option("-n,--n-octaves", n_octaves,
                        "Set the number of octaves in SURF, larger features "
                        "required larger values",
                        /*defaulted=*/true)
            ->check(CLI::Range(1, 10));
        cmd->add_option("-l,--octave-layers", octave_layers,
                        "Set the number of octaver layers",
                        /*defaulted=*/true)
            ->check(CLI::Range(1, 10));

        cmd->add_flag("-e,--extended-descriptor", extended,
                      "Calculate the extended 128-element descriptor instead "
                      "of 64 elements");
        cmd->add_flag("-u,--upright", upright,
                      "Don't calculate the orientation of the features "
                      "which might suffice in some instances and results "
                      "in much faster computation");
    }

    std::string out_path;
    int         hessian_threshold = 500;
    int         n_octaves         = 4;
    int         octave_layers     = 3;
    bool        extended          = false;
    bool        upright           = false;
};

struct SIFTArgs {
    SIFTArgs(CLI::App* cmd) {
        cmd->add_option("-o,--output", out_path,
                        "Output file-pattern for sift-features")
            ->required();
        cmd->add_option("-c,--feature-count", feature_count,
                        "Number of features to retain after ranking, 0 means "
                        "every feature is kept",
                        /*defaulted=*/true)
            ->check(CLI::Range(0, 10'000));
        cmd->add_option("-l,--octave-layers", octave_layers,
                        "Set the number of octaver layers",
                        /*defaulted=*/true)
            ->check(CLI::Range(1, 10));

        cmd->add_option("-t,--contrast-threshold", contrast_threshold,
                        "Threshold in contrast to filter weak features, HIGHER "
                        "value means LESS features",
                        /*defaulted=*/true)
            ->check(CLI::Range(0., 1.));
        cmd->add_option("-e,--edge-threshold", edge_threshold,
                        "Threshold to filter out edge-like features, HIGHER "
                        "value means MORE features",
                        /*defaulted=*/true)
            ->check(CLI::Range(0., 100.));
        cmd->add_option("-s,--sigma", sigma,
                        "Sigma of gaussian blur applied to the layers. Soft "
                        "images might choose smaller values",
                        /*defaulted=*/true)
            ->check(CLI::Range(0., 10.));
    }

    std::string out_path;
    int         feature_count = 0;
    int         octave_layers = 3;
    // NOLINTNEXTLINE(cppcoreguidelines-magic-numbers)
    double contrast_threshold = 0.04;
    double edge_threshold     = 10.;  // NOLINT(cppcoreguidelines-magic-numbers)
    double sigma              = 1.6;  // NOLINT(cppcoreguidelines-magic-numbers)
};
}  // namespace

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
    app.require_subcommand(1);  // Expect one or more feature commands
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
    const SIFTArgs sift(sift_cmd);

    CLI::App* surf_cmd = app.add_subcommand("surf", "Detect SURF features");
    surf_cmd->footer("\n\n");
    const SURFArgs surf(surf_cmd);

    CLI11_PARSE(app, argc, argv);

    using cv::Feature2D;
    using cv::Ptr;


    // TODO:
    // - Transform to a vector of commands, {Feature, OutPath}
    // - Write Result Files {Keypoints, Descriptor, Configuration,
    //   In-File(relative path)}
    // - Add Visualizer-Functionality? -> separate program that display
    //   different features on the original image
    auto [feature, out_path] = [&]() -> pair<Ptr<Feature2D>, string> {
        using cv::xfeatures2d::SIFT;
        using cv::xfeatures2d::SURF;

        if (*surf_cmd)
            return make_pair(SURF::create(surf.hessian_threshold,
                                          surf.n_octaves, surf.octave_layers,
                                          surf.extended, surf.upright),
                             surf.out_path);

        if (*sift_cmd)
            return make_pair(SIFT::create(sift.feature_count,
                                          sift.octave_layers,
                                          sift.contrast_threshold,
                                          sift.edge_threshold, sift.sigma),
                             sift.out_path);

        UNREACHABLE("provided unexpected subcommand");  // LCOV_EXCL_LINE
    }();

    batch_extractor extractor(feature, arg_input_files, out_path);
    const bool      success = extractor.process_batch(start_idx, end_idx);

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