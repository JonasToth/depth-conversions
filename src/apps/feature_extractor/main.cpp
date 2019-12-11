#include <CLI/CLI.hpp>
#include <iostream>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <stdexcept>
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
    using namespace std;

    CLI::App app{"Batch-processing tool to extract visual features"};
    app.require_subcommand();  // Expect one or more filter commands
    app.footer("\n\n"
               "An example invocation of the tool is:\n"
               "\n"
               "feature_extractor \\\n"
               "\n");

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    app.add_flag_function("-v,--version", apps::print_version(argv[0]),
                          "Print version and exit");

    CLI11_PARSE(app, argc, argv);

    return 0;
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
