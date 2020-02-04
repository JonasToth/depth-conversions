#include "keypoint_distribution.h"
#include "matching.h"
#include "min_dist.h"
#include "precision_recall.h"

#include <CLI/CLI.hpp>
#include <boost/histogram.hpp>
#include <opencv2/core/base.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/batch_visitor.h>
#include <util/colored_parse.h>
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
    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index for processing.")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index for processing.")
        ->required();

    CLI::App* cmd_keypoint_dist = app.add_subcommand(
        "keypoint-distribution",
        "Determine the keypoint distribution over all images");
    unsigned int image_width;
    cmd_keypoint_dist
        ->add_option("--image-width", image_width,
                     "The width of the original input images in pixel (check "
                     "the intrinsic!")
        ->required()
        ->check(CLI::Range(65'535));
    unsigned int image_height;
    cmd_keypoint_dist
        ->add_option("--image-height", image_height,
                     "The height of the original input images in pixel (check "
                     "the intrinsic!")
        ->required()
        ->check(CLI::Range(65'535));

    CLI::App* cmd_min_dist = app.add_subcommand(
        "min-distance", "Calculate the minimum distance of descriptors within "
                        "one image and analyze that.");
    string norm_name = "L2";
    cmd_min_dist->add_set("-n,--norm", norm_name,
                          {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                          "Set the norm that shall be used as distance measure",
                          /*defaulted=*/true);

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


    CLI::App* cmd_prec_rec = app.add_subcommand(
        "precision-recall",
        "Calculate precision and recall for consecutive image matching");
    string depth_image_path;
    cmd_prec_rec
        ->add_option("--depth-image", depth_image_path,
                     "File pattern for the original depth images")
        ->required();
    string pose_file_pattern;
    cmd_prec_rec
        ->add_option("--pose-file", pose_file_pattern,
                     "File pattern for the poses of each camera-idx.")
        ->required();
    string intrinsic_file;
    cmd_prec_rec
        ->add_option("--intrinsic", intrinsic_file,
                     "File path to the intrinsic - currently only pinhole!")
        ->required();
    optional<string> mask_file;
    cmd_prec_rec->add_option(
        "--mask", mask_file,
        "Image-mask with intrinsic dimenstion. Every black pixel "
        "means the camera has no vision there. White means, the "
        "camera sees these pixels. Use for distortion masking."
        "(8-bit grayscale png!)");
    cmd_prec_rec->add_set("-d,--match-norm", norm_name,
                          {"L1", "L2", "L2SQR", "HAMMING", "HAMMING2"},
                          "Set the norm that shall be used as distance measure",
                          /*defaulted=*/true);
    optional<string> backproject_pattern;
    CLI::Option*     backproject_opt = cmd_prec_rec->add_option(
        "--backprojection", backproject_pattern,
        "Provide a file-pattern to optionally print the "
        "backprojection for matched keypoints");
    CLI::Option* orig_imgs =
        cmd_prec_rec
            ->add_option("--orig-images", original_images,
                         "Provide the file pattern for the original image "
                         "the features were calculated on. Must be provided "
                         "for plotting.")
            ->needs(backproject_opt);
    backproject_opt->needs(orig_imgs);

    COLORED_APP_PARSE(app, argc, argv);

    if (*cmd_min_dist) {
        return analyze_min_distance(feature_file_input_pattern, start_idx,
                                    end_idx, str_to_norm(norm_name));
    }

    if (*cmd_keypoint_dist)
        return analyze_keypoint_distribution(feature_file_input_pattern,
                                             start_idx, end_idx, image_width,
                                             image_height);

    if (*cmd_matcher)
        return analyze_matching(feature_file_input_pattern, start_idx, end_idx,
                                str_to_norm(norm_name), !no_crosscheck,
                                match_output, original_images);

    if (*cmd_prec_rec)
        return analyze_precision_recall(
            feature_file_input_pattern, start_idx, end_idx, depth_image_path,
            pose_file_pattern, intrinsic_file, mask_file,
            str_to_norm(norm_name), backproject_pattern, original_images);

    UNREACHABLE("Expected to end program with "  // LCOV_EXCL_LINE
                "subcommand processing");        // LCOV_EXCL_LINE
}
MAIN_TAIL
