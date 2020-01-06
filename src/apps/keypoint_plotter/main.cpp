#include <CLI/CLI.hpp>
#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/colored_parse.h>
#include <util/tool_macro.h>
#include <util/version_printer.h>
#include <vector>


/// \ingroup feature-plotter-driver
enum class feature_color { green, blue, red, orange, purple, all };

/// \ingroup feature-plotter-driver
struct color_to_rgb {
    static cv::Scalar convert(feature_color c) {
        using cv::Scalar;
        switch (c) {
        case feature_color::green: return Scalar(0, 255, 0);
        case feature_color::blue: return Scalar(255, 0, 0);
        case feature_color::red: return Scalar(0, 0, 255);
        case feature_color::orange: return Scalar(0, 255, 255);
        case feature_color::purple: return Scalar(255, 0, 255);
        case feature_color::all: return Scalar::all(-1);
        }
        UNREACHABLE("Invalid enum-value!");  // LCOV_EXCL_LINE
    }
};

/// \defgroup feature-plotter-driver plot detectede keypoints and matches.
///
/// This driver utilizes OpenCV for keypoint and matching plotting.

/// Parallelized driver to batch-process images for keypoint plotting and to
/// plot matches.
/// \ingroup feature-plotter-driver
/// \returns 0 if all images could be processed, 1 if any image fails
MAIN_HEAD("Batch-processing tool to plot keypoints and matches") {
    COLORED_APP_PARSE(app, argc, argv);

    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    std::string input_file =
        "/home/jonas/owncloud/Freiberg/Masterarbeit/full_data_sets/lehrpfad/"
        "test_text_out/default-0000.orb";

    try {
        using cv::FileNode;
        using cv::FileStorage;
        using cv::read;

        FileStorage fs{input_file,
                       FileStorage::READ | FileStorage::FORMAT_YAML};

        // if 'keypoints'-key does not exist give an error.
        const FileNode            keypoints_node = fs["keypoints"];
        std::vector<cv::KeyPoint> keypoints;
        read(keypoints_node, keypoints);

        if (keypoints.empty()) {
            std::cerr << "WARNING: No keypoints loaded!\n";
        }

        std::string original_image =
            "/home/jonas/owncloud/Freiberg/Masterarbeit/full_data_sets/"
            "lehrpfad/filtered_depth/flexion-0000.png";
        if (original_image.empty()) {
            const FileNode file_path_node = fs["source_path"];
            read(file_path_node, original_image, "");
        }

        if (original_image.empty()) {
            std::cerr << "No image-path found to plot the keypoints on!\n";
            return 1;
        }

        {
            const std::string output_file =
                "/home/jonas/owncloud/Freiberg/Masterarbeit/full_data_sets/"
                "lehrpfad/test_text_out/default-0000.png";
            const cv::Mat source_image =
                cv::imread(original_image, cv::IMREAD_UNCHANGED);
            cv::Mat converted_source;
            source_image.convertTo(converted_source, CV_8UC1, 1. / 255.);

            cv::Mat       img_keypoints_drawn;
            const feature_color color = feature_color::purple;
            cv::drawKeypoints(converted_source, keypoints, img_keypoints_drawn,
                              color_to_rgb::convert(color),
                              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            const bool write_success =
                cv::imwrite(output_file, img_keypoints_drawn);

            if (!write_success) {
                std::cerr << "Did not write image!\n";
                return 1;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Shit happens\n" << e.what() << "\n";
    } catch (...) {
        std::cerr << "Shit happens\n";
    }

    return 0;
}
MAIN_TAIL
