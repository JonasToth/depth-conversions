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

struct CommonArgs {
    CommonArgs(CLI::App* cmd) {
        cmd->add_option("-o,--output", out_path,
                        "Output file-pattern for sift-features")
            ->required();
    }

    std::string out_path;
};

struct SURFArgs : CommonArgs {
    SURFArgs(CLI::App* cmd)
        : CommonArgs(cmd) {
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

    int  hessian_threshold = 500;
    int  n_octaves         = 4;
    int  octave_layers     = 3;
    bool extended          = false;
    bool upright           = false;
};

struct SIFTArgs : CommonArgs {
    SIFTArgs(CLI::App* cmd)
        : CommonArgs(cmd) {
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

    int    feature_count      = 0;
    int    octave_layers      = 3;
    double contrast_threshold = 0.04;
    double edge_threshold     = 10.;
    double sigma              = 1.6;
};

struct ORBArgs : CommonArgs {
    ORBArgs(CLI::App* cmd)
        : CommonArgs(cmd) {
        cmd->add_option("-c,--feature-count", feature_count,
                        "Number of features to retain after ranking, 0 means "
                        "every feature is kept",
                        /*defaulted=*/true)
            ->check(CLI::Range(0, 10'000));
        cmd->add_option("-s,--scale-factor", scale_factor,
                        "Scale-Factor for the image pyramid. Value == 2. means "
                        "that the size is halfed",
                        /*defaulted=*/true)
            ->check(CLI::Range(1., 4.));
        cmd->add_option("-n,--n-levels", n_levels, "Number of pyramid levels",
                        /*defaulted=*/true);
        cmd->add_option("-t,--edge-threshold", edge_threshold,
                        "size of the border where features are not detected, "
                        "should match patch_size",
                        /*defaulted=*/true);
        cmd->add_option(
            "-l,--first-level", first_level,
            "Level of the pyramid to put the source image to. Previous layers "
            "are filled with upscaled versions of the image.",
            /*defaulted=*/true);
        cmd->add_option("-w,--wta-k", WTA_K,
                        "number of points that produce each element of the "
                        "oriented BRIEF descriptor",
                        /*defaulted=*/true);
        cmd->add_set("-m,--score-metric", score_type, {"HARRIS", "FAST"},
                     "Scoretype that is used as metric", /*defaulted=*/true);
        cmd->add_option("-p,--patch-size", path_size,
                        "size of the patch used by the BRIEF descriptor",
                        /*defaulted=*/true);
        cmd->add_option("-f,--fast-threshold", fast_threshold,
                        "the FAST threshold", /*defaulted=*/true);
    }

    static cv::ORB::ScoreType
    string_to_score_type(std::string_view picked) noexcept {
        Expects(!picked.empty());
        // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CHOICE(enumerator)                                                     \
    if (picked == #enumerator)                                                 \
        return cv::ORB::ScoreType::enumerator##_SCORE;
        CHOICE(FAST)
        CHOICE(HARRIS)
#undef CHOICE
        UNREACHABLE("Invalid choice for ScoreType!");  // LCOV_EXCL_LINE
    }

    int         feature_count  = 700;
    float       scale_factor   = 1.2F;
    int         n_levels       = 8;
    int         edge_threshold = 31;
    int         first_level    = 0;
    int         WTA_K          = 2;
    std::string score_type     = "HARRIS";
    int         path_size      = 31;
    int         fast_threshold = 20;
};

struct AKAZEArgs : CommonArgs {
    AKAZEArgs(CLI::App* cmd)
        : CommonArgs(cmd) {
        cmd->add_set("-d,--descriptor-type", descriptor_type,
                     {"KAZE_UPRIGHT", "KAZE", "MLDB_UPRIGHT", "MLDB"},
                     "Which descriptor to use, upright means that no "
                     "orientation is computed",
                     /*defaulted=*/true);
        cmd->add_option("-s,--descriptor-size", descriptor_size,
                        "Size of the descriptor in bits, 0 => full size",
                        /*defaulted=*/true)
            ->check(CLI::Range(0, 1'000'000));
        cmd->add_option("-c,--descriptor-channels", descriptor_channels,
                        "Number of channels in the descriptor",
                        /*defaulted=*/true)
            ->check(CLI::Range(1, 3));
        cmd->add_option("-t,--threshold", threshold,
                        "Detector reponse threshold to accept point",
                        /*defaulted=*/true);
        cmd->add_option("-n,--n-octaves", n_octaves,
                        "Maximum octave evolution of the image",
                        /*defaulted=*/true);
        cmd->add_option("-l,--n-octave-layers", n_octave_layers,
                        "Default number of sublevels per scale level",
                        /*defaulted=*/true);
    }

    static cv::AKAZE::DescriptorType
    string_to_descriptor(std::string_view picked) noexcept {
        Expects(!picked.empty());
        // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CHOICE(enumerator)                                                     \
    if (picked == #enumerator)                                                 \
        return cv::AKAZE::DescriptorType::DESCRIPTOR_##enumerator;
        CHOICE(KAZE_UPRIGHT)
        CHOICE(KAZE)
        CHOICE(MLDB_UPRIGHT)
        CHOICE(MLDB)
#undef CHOICE
        UNREACHABLE("Invalid Descriptor Choice detected!");  // LCOV_EXCL_LINE
    }

    static cv::KAZE::DiffusivityType
    string_to_diffusivity(std::string_view picked) noexcept {
        Expects(!picked.empty());
        // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CHOICE(enumerator)                                                     \
    if (picked == #enumerator)                                                 \
        return cv::KAZE::DiffusivityType::DIFF_##enumerator;
        CHOICE(WEICKERT)
        CHOICE(CHARBONNIER)
        CHOICE(PM_G1)
        CHOICE(PM_G2)
#undef CHOICE
        UNREACHABLE("Invalid Diffusivity Choice detected!");  // LCOV_EXCL_LINE
    }

    std::string descriptor_type     = "MLDB";
    int         descriptor_size     = 0;
    int         descriptor_channels = 3;
    float       threshold           = 0.001F;
    int         n_octaves           = 4;
    int         n_octave_layers     = 4;
    std::string diffusivity         = "PM_G2";
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
    app.add_flag_function("-v,--version", apps::print_version(*argv),
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

    CLI::App* orb_cmd = app.add_subcommand("orb", "Detect ORB features");
    orb_cmd->footer("\n\n");
    const ORBArgs orb(orb_cmd);

    CLI::App* akaze_cmd = app.add_subcommand("akaze", "Detect AKAZE features");
    akaze_cmd->footer("\n\n");
    const AKAZEArgs akaze(akaze_cmd);

    CLI11_PARSE(app, argc, argv);

    // TODO:
    // - Transform to a vector of commands, {Feature, OutPath}
    // - Write Result Files {Keypoints, Descriptor, Configuration,
    //   In-File(relative path)}
    // - Add Visualizer-Functionality? -> separate program that display
    //   different features on the original image
    auto detector = [&]() -> Detector {
        using cv::xfeatures2d::SIFT;
        using cv::xfeatures2d::SURF;
        using cv::ORB;
        using cv::AKAZE;

        if (*surf_cmd)
            return {SURF::create(surf.hessian_threshold, surf.n_octaves,
                                 surf.octave_layers, surf.extended,
                                 surf.upright),
                    surf.out_path, feature_color::orange};

        if (*sift_cmd)
            return {SIFT::create(sift.feature_count, sift.octave_layers,
                                 sift.contrast_threshold, sift.edge_threshold,
                                 sift.sigma),
                    sift.out_path, feature_color::green};

        if (*orb_cmd)
            return {ORB::create(orb.feature_count, orb.scale_factor,
                                orb.n_levels, orb.edge_threshold,
                                orb.first_level, orb.WTA_K,
                                ORBArgs::string_to_score_type(orb.score_type),
                                orb.path_size, orb.fast_threshold),
                    orb.out_path, feature_color::red};

        if (*akaze_cmd)
            return {AKAZE::create(
                        AKAZEArgs::string_to_descriptor(akaze.descriptor_type),
                        akaze.descriptor_size, akaze.descriptor_channels,
                        akaze.threshold, akaze.n_octaves, akaze.n_octave_layers,
                        AKAZEArgs::string_to_diffusivity(akaze.diffusivity)),
                    akaze.out_path, feature_color::blue};

        UNREACHABLE("provided unexpected subcommand");  // LCOV_EXCL_LINE
    }();

    batch_extractor extractor(detector, arg_input_files);
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
