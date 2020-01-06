#include <CLI/CLI.hpp>
#include <iostream>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
#include <string>
#include <util/colored_parse.h>
#include <util/version_printer.h>
#include <vector>

/// \defgroup feature-plotter-driver plot detectede keypoints and matches.
///
/// This driver utilizes OpenCV for keypoint and matching plotting.

/// Parallelized driver to batch-process images for keypoint plotting and to
/// plot matches.
/// \ingroup feature-plotter-driver
/// \returns 0 if all images could be processed, 1 if any image fails
int main(int argc, char** argv) try {
    using namespace sens_loc;
    using namespace std;
    CLI::App app{"Batch-processing tool to filter depth images and maps"};
    COLORED_APP_PARSE(app, argc, argv);

    // Explicitly disable threading from OpenCV functions, as the
    // parallelization is done at a higher level.
    // That means, that each filter application is not multithreaded, but each
    // image modification is. This is necessary as "TaskFlow" does not play
    // nice with OpenCV threading and they introduce data races in the program
    // because of that.
    cv::setNumThreads(0);

    return 0;
} catch (const std::exception& e) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n"
              << "Message: " << e.what() << "\n";
    return 1;
} catch (...) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n";
    return 1;
}
