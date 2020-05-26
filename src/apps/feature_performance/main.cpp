#include "keypoint_distribution.h"
#include "matching.h"
#include "min_dist.h"
#include "recognition_performance.h"

#include <CLI/CLI.hpp>
#include <boost/histogram.hpp>
#include <opencv2/core/base.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/batch_visitor.h>
#include <util/colored_parse.h>
#include <util/common_structures.h>
#include <util/tool_macro.h>
#include <util/version_printer.h>

static cv::NormTypes str_to_norm(std::string_view n) {

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
    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    // Require exactly one subcommand.
    app.require_subcommand(1);

    string feature_file_input_pattern;
    app.add_option("-i,--input", feature_file_input_pattern,
                   "Define file-pattern for the feature files to be plotted")
        ->required();
    int start_idx = 0;
    app.add_option("-s,--start", start_idx, "Start index for processing.")
        ->required();
    int end_idx = 0;
    app.add_option("-e,--end", end_idx, "End index for processing.")
        ->required();
    optional<string> statistics_file;
    app.add_option(
        "-o,--output", statistics_file,
        "Write the result of the analysis into a yaml-file instead to stdout");

    CLI::App* cmd_keypoint_dist = app.add_subcommand(
        "keypoint-distribution",
        "Determine the keypoint distribution over all images");
    unsigned int image_width = 0;
    cmd_keypoint_dist
        ->add_option("--image-width", image_width,
                     "The width of the original input images in pixel (check "
                     "the intrinsic!")
        ->required()
        ->check(CLI::Range(65'535));
    unsigned int image_height = 0;
    cmd_keypoint_dist
        ->add_option("--image-height", image_height,
                     "The height of the original input images in pixel (check "
                     "the intrinsic!")
        ->required()
        ->check(CLI::Range(65'535));
    optional<string> response_histo;
    cmd_keypoint_dist->add_option(
        "--response-histo", response_histo,
        "Filepath where the keypoint response histogram shall be written to.");
    optional<string> size_histo;
    cmd_keypoint_dist->add_option(
        "--size-histo", size_histo,
        "Filepath where the keypoint size histogram shall be written to.");
    optional<string> kp_distance_histo;
    cmd_keypoint_dist->add_option(
        "--kp-distance-histo", kp_distance_histo,
        "Filepath where the histogram of the distance to the nearest neighbour "
        "of the keypoints shall be written to.");
    optional<string> kp_distribution_histo;
    cmd_keypoint_dist->add_option(
        "--kp-distribution-histo", kp_distribution_histo,
        "Filepath where the histogram of the distribution of the keypoints "
        "shall be written to.");

    CLI::App* cmd_min_dist = app.add_subcommand(
        "min-distance", "Calculate the minimum distance of descriptors within "
                        "one image and analyze that.");
    string norm_name = "L2";
    cmd_min_dist->add_set("-n,--norm", norm_name,
                          {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                          "Set the norm that shall be used as distance measure",
                          /*defaulted=*/true);
    optional<string> min_distance_histo;
    cmd_min_dist->add_option(
        "--min-distance-histo", min_distance_histo,
        "Write the histogram of minimal descriptor distance to this file");

    CLI::App* cmd_matcher = app.add_subcommand(
        "matching",
        "Analyze the matchability of the descriptors with consecutive images.");
    cmd_matcher->add_set("-d,--distance-norm", norm_name,
                         {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                         "Set the norm that shall be used as distance measure",
                         /*defaulted=*/true);
    bool no_crosscheck = false;
    cmd_matcher->add_flag("--no-crosscheck", no_crosscheck,
                          "Disable crosschecking");
    optional<string> match_output;
    CLI::Option*     match_output_opt = cmd_matcher->add_option(
        "--match-output", match_output,
        "Provide a filename for drawing the matches onto an image.");
    optional<string> original_images;
    CLI::Option*     orig_imgs_opt =
        cmd_matcher
            ->add_option("--original-images", original_images,
                         "Provide the file pattern for the original image "
                         "the features were calculated on. Must be provided "
                         "for plotting.")
            ->needs(match_output_opt);
    match_output_opt->needs(orig_imgs_opt);
    optional<string> matched_distance_histo;
    cmd_matcher->add_option(
        "--matched-distance-histo", matched_distance_histo,
        "Write histogram data of the descriptor distance of matches");


    CLI::App* cmd_rec_perf = app.add_subcommand(
        "recognition-performance",
        "Calculate precision and recall for consecutive image matching");
    string depth_image_path;
    cmd_rec_perf
        ->add_option("--depth-image", depth_image_path,
                     "File pattern for the original depth images")
        ->required();
    string pose_file_pattern;
    cmd_rec_perf
        ->add_option("--pose-file", pose_file_pattern,
                     "File pattern for the poses of each camera-idx.")
        ->required();
    string intrinsic_file;
    cmd_rec_perf
        ->add_option("--intrinsic", intrinsic_file,
                     "File path to the intrinsic - currently only pinhole!")
        ->required();
    optional<string> mask_file;
    cmd_rec_perf->add_option(
        "--mask", mask_file,
        "Image-mask with intrinsic dimenstion. Every black pixel "
        "means the camera has no vision there. White means, the "
        "camera sees these pixels. Use for distortion masking."
        "(8-bit grayscale png!)");
    cmd_rec_perf->add_set("-d,--match-norm", norm_name,
                          {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                          "Set the norm that shall be used as distance measure",
                          /*defaulted=*/true);
    float keypoint_distance_threshold = 3.0F;
    cmd_rec_perf->add_option("--keypoint-distance-threshold",
                             keypoint_distance_threshold,
                             "Threshold for the reprojection error of "
                             "keypoints to be considered a correspondence",
                             /*defaulted=*/true);
    optional<string> backproject_pattern;
    CLI::Option*     backproject_opt = cmd_rec_perf->add_option(
        "--backprojection", backproject_pattern,
        "Provide a file-pattern to optionally print the "
        "backprojection for matched keypoints");
    CLI::Option* orig_imgs =
        cmd_rec_perf
            ->add_option("--orig-images", original_images,
                         "Provide the file pattern for the original image "
                         "the features were calculated on. Must be provided "
                         "for plotting.")
            ->needs(backproject_opt);
    backproject_opt->needs(orig_imgs);

    unsigned int tp_strength = 6;
    cmd_rec_perf
        ->add_option("--true-positive-strength", tp_strength,
                     "Line strength to connect two true positive keypoints",
                     /*defaulted=*/true)
        ->needs(backproject_opt);
    std::vector<unsigned char> tp_rgb{65U, 117U, 5U};
    cmd_rec_perf
        ->add_option("--true-positive-rgb", tp_rgb,
                     "RGB values [0-255] for true positive line color",
                     /*defaulted=*/true)
        ->expected(3)
        ->needs(backproject_opt);

    unsigned int fn_strength = 6;
    cmd_rec_perf
        ->add_option("--false-negative-strength", fn_strength,
                     "Line strength to connect two false negative keypoints",
                     /*defaulted=*/true)
        ->needs(backproject_opt);
    std::vector<unsigned char> fn_rgb{144U, 19U, 254U};
    cmd_rec_perf
        ->add_option("--false-negative-rgb", fn_rgb,
                     "RGB values [0-255] for false negative line color",
                     /*defaulted=*/true)
        ->expected(3)
        ->needs(backproject_opt);

    unsigned int fp_strength = 1;
    cmd_rec_perf
        ->add_option("--false-positive-strength", fn_strength,
                     "Line strength to connect two false positive keypoints",
                     /*defaulted=*/true)
        ->needs(backproject_opt);
    std::vector<unsigned char> fp_rgb{245U, 166U, 35U};
    cmd_rec_perf
        ->add_option("--false-positive-rgb", fp_rgb,
                     "RGB values [0-255] for false positive line color",
                     /*defaulted=*/true)
        ->expected(3)
        ->needs(backproject_opt);

    optional<string> backprojection_selected_histo;
    cmd_rec_perf->add_option("--backprojection-selected-histo",
                             backprojection_selected_histo,
                             "File for the histogram of the backprojection "
                             "error of the selected elements");
    optional<string> relevant_histo;
    cmd_rec_perf->add_option(
        "--relevant-elements-histo", relevant_histo,
        "File for the histogram for the number of relevant elements per frame");
    optional<string> true_positive_histo;
    cmd_rec_perf->add_option(
        "--true-positive-histo", true_positive_histo,
        "File for the histogram for the number of true positives per frame.");
    optional<string> false_positive_histo;
    cmd_rec_perf->add_option(
        "--false-positive-histo", false_positive_histo,
        "File for the histogram for the number of false postives per frame.");
    optional<string> true_positive_distance_histo;
    cmd_rec_perf->add_option("--true-positive-distance-histo",
                             true_positive_distance_histo,
                             "File for the histogram of the descriptor "
                             "distance for true positives.");
    optional<string> false_positive_distance_histo;
    cmd_rec_perf->add_option("--false-positive-distance-histo",
                             false_positive_distance_histo,
                             "File for the histogram of the descriptor "
                             "distance for false positives.");

    COLORED_APP_PARSE(app, argc, argv);

    util::processing_input in{feature_file_input_pattern, start_idx, end_idx};

    if (*cmd_min_dist) {
        return analyze_min_distance(in, str_to_norm(norm_name), statistics_file,
                                    min_distance_histo);
    }

    if (*cmd_keypoint_dist)
        return analyze_keypoint_distribution(
            in, image_width, image_height, statistics_file, response_histo,
            size_histo, kp_distance_histo, kp_distribution_histo);

    if (*cmd_matcher)
        return analyze_matching(in, str_to_norm(norm_name), !no_crosscheck,
                                statistics_file, matched_distance_histo,
                                match_output, original_images);

    if (*cmd_rec_perf) {
        recognition_analysis_input rec_in{
            /*depth_image_pattern=*/depth_image_path,
            /*pose_file_pattern=*/pose_file_pattern,
            /*intrinsic_file=*/intrinsic_file,
            /*mask_file=*/mask_file,
            /*matching_norm=*/str_to_norm(norm_name),
            /*keypoint_distance_threshold=*/keypoint_distance_threshold};
        recognition_analysis_output_options out_opts{
            /*backproject_pattern=*/backproject_pattern,
            /*original_files=*/original_images,
            /*stat_file=*/statistics_file,
            /*backprojection_selected_histo=*/backprojection_selected_histo,
            /*relevant_histo=*/relevant_histo,
            /*true_positive_histo=*/true_positive_histo,
            /*false_positive_histo=*/false_positive_histo,
            /*true_positive_distance_histo=*/true_positive_distance_histo,
            /*false_positive_distance_histo=*/false_positive_distance_histo};
        backproject_style tp_style(tp_rgb[0], tp_rgb[1], tp_rgb[2],
                                   gsl::narrow<int>(tp_strength));
        backproject_style fn_style(fn_rgb[0], fn_rgb[1], fn_rgb[2],
                                   gsl::narrow<int>(fn_strength));
        backproject_style fp_style(fp_rgb[0], fp_rgb[1], fp_rgb[2],
                                   gsl::narrow<int>(fp_strength));
        return analyze_recognition_performance(in, rec_in, out_opts,
                                               {tp_style, fn_style, fp_style});
    }

    UNREACHABLE("Expected to end program with "  // LCOV_EXCL_LINE
                "subcommand processing");        // LCOV_EXCL_LINE
}
MAIN_TAIL
