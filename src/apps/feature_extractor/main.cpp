#include "batch_extractor.h"

#include <CLI/CLI.hpp>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <sens_loc/util/overloaded.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <util/colored_parse.h>
#include <util/tool_macro.h>
#include <util/version_printer.h>
#include <variant>
#include <vector>

/// \defgroup feature-extractor-driver Feature detection and extraction
///
/// This driver utilizes OpenCV for feature detection, extraction and
/// serialization.

namespace {

/// \ingroup feature-extractor-driver
struct SURFArgs {
    SURFArgs() = default;
    SURFArgs(CLI::App* cmd) {
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

/// \ingroup feature-extractor-driver
struct SIFTArgs {
    SIFTArgs() = default;
    SIFTArgs(CLI::App* cmd) {
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

/// \ingroup feature-extractor-driver
struct ORBArgs {
    ORBArgs() = default;
    ORBArgs(CLI::App* cmd) {
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

/// \ingroup feature-extractor-driver
struct AKAZEArgs {
    AKAZEArgs() = default;
    AKAZEArgs(CLI::App* cmd) {
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

/// Helper enum to provide information on the capability of an algorithm.
/// This is used for registering the descriptors and detectors.
/// \ingroup feature-extractor-driver
enum class capability : uint8_t {
    none     = 0,
    detect   = 1U << 0U,
    describe = 1U << 1U,
};
constexpr capability operator|(capability element1,
                               capability element2) noexcept {
    using T = std::underlying_type_t<capability>;
    return static_cast<capability>(static_cast<T>(element1) |
                                   static_cast<T>(element2));
}
constexpr capability operator&(capability element1,
                               capability element2) noexcept {
    using T = std::underlying_type_t<capability>;
    return static_cast<capability>(static_cast<T>(element1) &
                                   static_cast<T>(element2));
}
}  // namespace

/// Parallelized driver to batch-process images for feature detection and
/// extraction.
/// \ingroup feature-extractor-driver
/// \returns 0 if all images could be processed, 1 if any image fails
MAIN_HEAD("Batch-processing tool to extract visual features") {
    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    app.require_subcommand(2);
    app.footer("\n\n"
               "An example invocation of the tool is:\n"
               "\n"
               "feature_extractor --input 'flexion-{}.png'            \\\n"
               "                  --output akaze-{:04d}.feature[.gz]  \\\n"
               "                  --start 0                           \\\n"
               "                  --end 100                           \\\n"
               "                  detector akaze                      \\\n"
               "                  descriptor akaze"
               "\n");

    std::string arg_input_files;
    app.add_option(
           "-i,--input", arg_input_files,
           "Input pattern for images to filter; e.g. \"flexion-{}.png\"")
        ->required();
    std::string arg_out_path;
    app.add_option("-o,--output", arg_out_path,
                   "Output file-pattern for the feature information")
        ->required();
    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index of batch, inclusive")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index of batch, inclusive")
        ->required();

    CLI::App* detector_cmd =
        app.add_subcommand("detector", "Configure the detector");
    std::optional<float> keypoint_size_threshold;
    detector_cmd->add_option(
        "--kp-size-threshold", keypoint_size_threshold,
        "Sets a minimum keypoint size to be considered as feasible keypoint");
    std::optional<float> keypoint_response_threshold;
    detector_cmd->add_option(
        "--kp-response-threshold", keypoint_response_threshold,
        "Sets a minimum response to be considered as feasible keypoint");
    detector_cmd->require_subcommand(1);

    CLI::App* descriptor_cmd = app.add_subcommand(
        "descriptor", "Configure the descriptor used for the keypoints");
    descriptor_cmd->require_subcommand(1);

    using feature_args = variant<SURFArgs, SIFTArgs, AKAZEArgs, ORBArgs>;
    unordered_map<CLI::App*, feature_args> detector_params;
    unordered_map<CLI::App*, feature_args> descriptor_params;

    for (auto&& a : vector<tuple<const char*, const char*, capability>>{{
             make_tuple("sift", "Use SIFT implementation",
                        capability::detect | capability::describe),
             make_tuple("surf", "Use SURF implementation",
                        capability::detect | capability::describe),
             make_tuple("orb", "Use ORB implementation",
                        capability::detect | capability::describe),
             make_tuple("akaze", "Use AKAZE implementation",
                        capability::detect | capability::describe),
         }}) {
        auto cmd_to_args = [&](string_view name,
                               CLI::App*   cmd) -> feature_args {
            if (name == "sift")
                return SIFTArgs{cmd};
            if (name == "surf")
                return SURFArgs{cmd};
            if (name == "orb")
                return ORBArgs{cmd};
            if (name == "akaze")
                return AKAZEArgs{cmd};
            UNREACHABLE(                                      // LCOV_EXCL_LINE
                "Unexpected detector/descriptor provided!");  // LCOV_EXCL_LINE
        };

        if ((get<2>(a) & capability::detect) != capability::none) {
            CLI::App* c = detector_cmd->add_subcommand(get<0>(a), get<1>(a));
            detector_params.emplace(make_pair(c, cmd_to_args(get<0>(a), c)));
        }

        if ((get<2>(a) & capability::describe) != capability::none) {
            CLI::App* c = descriptor_cmd->add_subcommand(get<0>(a), get<1>(a));
            descriptor_params.emplace(make_pair(c, cmd_to_args(get<0>(a), c)));
        }
    }

    COLORED_APP_PARSE(app, argc, argv);

    Ensures(descriptor_cmd->get_subcommands().size() == 1);
    Ensures(detector_cmd->get_subcommands().size() == 1);

    // Create a vector of functors (unique_ptr<filter_interface>) that will
    // be executed in order.
    // Each element is created by one subcommand and its parameters.
    using namespace sens_loc::util;
    using cv::AKAZE;
    using cv::ORB;
    using cv::xfeatures2d::SIFT;
    using cv::xfeatures2d::SURF;

    auto argument_visitor = overloaded{
        [](const SIFTArgs& sift) -> cv::Ptr<cv::Feature2D> {
            return SIFT::create(sift.feature_count, sift.octave_layers,
                                sift.contrast_threshold, sift.edge_threshold,
                                sift.sigma);
        },
        [](const SURFArgs& surf) -> cv::Ptr<cv::Feature2D> {
            return SURF::create(surf.hessian_threshold, surf.n_octaves,
                                surf.octave_layers, surf.extended,
                                surf.upright);
        },
        [](const ORBArgs& orb) -> cv::Ptr<cv::Feature2D> {
            return ORB::create(orb.feature_count, orb.scale_factor,
                               orb.n_levels, orb.edge_threshold,
                               orb.first_level, orb.WTA_K,
                               ORBArgs::string_to_score_type(orb.score_type),
                               orb.path_size, orb.fast_threshold);
        },
        [](const AKAZEArgs& akaze) -> cv::Ptr<cv::Feature2D> {
            return AKAZE::create(
                AKAZEArgs::string_to_descriptor(akaze.descriptor_type),
                akaze.descriptor_size, akaze.descriptor_channels,
                akaze.threshold, akaze.n_octaves, akaze.n_octave_layers,
                AKAZEArgs::string_to_diffusivity(akaze.diffusivity));
        },
    };

    CLI::App* provided_detector_cmd = detector_cmd->get_subcommands()[0];
    Ensures(detector_params.count(provided_detector_cmd) == 1);

    CLI::App* provided_descriptor_cmd = descriptor_cmd->get_subcommands()[0];
    Ensures(descriptor_params.count(provided_descriptor_cmd) == 1);

    const feature_args& det_args{detector_params[provided_detector_cmd]};
    const feature_args& desc_args{descriptor_params[provided_descriptor_cmd]};

    optional<batch_extractor::filter_func> filter;

    if (keypoint_size_threshold && !keypoint_response_threshold)
        filter = [s = *keypoint_size_threshold](const cv::KeyPoint& kp) {
            return kp.size < s;
        };

    else if (!keypoint_size_threshold && keypoint_response_threshold)
        filter = [r = *keypoint_response_threshold](const cv::KeyPoint& kp) {
            return kp.response < r;
        };

    else if (keypoint_size_threshold && keypoint_response_threshold)
        filter = [s = *keypoint_size_threshold,
                  r = *keypoint_response_threshold](const cv::KeyPoint& kp) {
            return kp.size < s || kp.response < r;
        };

    const batch_extractor extractor(visit(argument_visitor, det_args),
                                    visit(argument_visitor, desc_args),
                                    arg_input_files, arg_out_path, filter);

    const bool success = extractor.process_batch(start_idx, end_idx);
    return success ? 0 : 1;
}
MAIN_TAIL
