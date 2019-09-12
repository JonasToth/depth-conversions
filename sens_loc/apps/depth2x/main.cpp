#include "converters.h"

#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <rang.hpp>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/util/console.h>
#include <sens_loc/version.h>
#include <stdexcept>
#include <string>


int main(int argc, char **argv) {
    using namespace sens_loc;
    using namespace std;

    CLI::App app{"Batchconversion of depth images to bearing angle images."};

    auto print_version = [argv](int /*count*/) {
        cout << argv[0] << " " << get_version() << "\n";
        exit(0);
    };
    app.add_flag_function("-v,--version", print_version,
                          "Print version and exit");

    string calibration_file;
    app.add_option("-c,--calibration", calibration_file,
                   "File that contains calibration parameters for the camera")
        ->required()
        ->check(CLI::ExistingFile);
    apps::file_patterns files;

    app.add_option("-i,--input", files.input,
                   "Input pattern for image, e.g. \"depth-{}.png\"")
        ->required();

    string input_type = "pinhole-depth";
    app.add_set("-t,--type", input_type, {"pinhole-depth", "pinhole-range"});

    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index of batch, inclusive")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index of batch, inclusive")
        ->required();

    // Bearing angle images territory
    CLI::App *bearing_cmd = app.add_subcommand(
        "bearing", "Converts depth images into bearing angle images");
    bearing_cmd->add_option(
        "--horizontal", files.horizontal,
        "Calculate horizontal bearing angle image and write to this pattern");
    bearing_cmd->add_option(
        "--vertical", files.vertical,
        "Calculate vertical bearing angle and write to this pattern");
    bearing_cmd->add_option(
        "--diagonal", files.diagonal,
        "Calculate diagonal bearing angle and write to this pattern");
    bearing_cmd->add_option(
        "--anti-diagonal", files.antidiagonal,
        "Calculate anti-diagonal bearing angle and write to this pattern");

    // Range images territory
    CLI::App *range_cmd = app.add_subcommand(
        "range", "Convert depth images into range images (laser-scan like)");
    range_cmd->add_option("-o,--output", files.output,
                          "Output pattern for the range images.");

    // curvature images territory
    CLI::App *mean_curv_cmd = app.add_subcommand(
        "mean-curvature", "Convert depth images into mean-curvature images");
    mean_curv_cmd->add_option("-o,--output", files.output,
                              "Output pattern for the mean-curvature images.");

    CLI::App *gauss_curv_cmd = app.add_subcommand(
        "gauss-curvature",
        "Convert depth images into gaussian-curvature images");
    gauss_curv_cmd->add_option(
        "-o,--output", files.output,
        "Output pattern for the gaussian-curvature images.");

    // Max-Curve images
    CLI::App *max_curve_cmd = app.add_subcommand(
        "max-curve", "Convert depth images into max-curve images");
    max_curve_cmd->add_option("-o,--output", files.output,
                              "Output pattern for the max-curve images.");

    // Flexion images
    CLI::App *flexion_cmd = app.add_subcommand(
        "flexion", "Convert depth images into flexion images");
    flexion_cmd->add_option("-o,--output", files.output,
                            "Output pattern for the flexion images.");

    CLI11_PARSE(app, argc, argv);

    if (files.input.empty()) {
        cerr << util::err{}
             << "Input pattern for files to process is required!\n";
        return 1;
    }

    // Options that are always required are checked first.
    ifstream                         calibration_fstream{calibration_file};
    optional<camera_models::pinhole> intrinsic =
        io::load_pinhole_intrinsic(calibration_fstream);

    if (!intrinsic) {
        cerr << util::err{};
        cerr << "Could not load intrinsic calibration \"" << rang::style::bold
             << calibration_file << rang::style::reset << "\"!\n";
        return 1;
    }

    try {
        unique_ptr<apps::batch_converter> c =
            [&]() -> unique_ptr<apps::batch_converter> {
            if (*bearing_cmd)
                return make_unique<apps::bearing_converter>(files, *intrinsic);
            if (*range_cmd)
                return make_unique<apps::range_converter>(files, *intrinsic);
            if (*mean_curv_cmd)
                return make_unique<apps::mean_curv_converter>(files,
                                                              *intrinsic);
            if (*gauss_curv_cmd)
                return make_unique<apps::gauss_curv_converter>(files,
                                                               *intrinsic);
            if (*max_curve_cmd)
                return make_unique<apps::max_curve_converter>(files,
                                                              *intrinsic);
            if (*flexion_cmd)
                return make_unique<apps::flexion_converter>(files, *intrinsic);

            throw std::invalid_argument{"target type for conversion required!"};
        }();
        return c->process_batch(start_idx, end_idx);
    } catch (const std::invalid_argument &e) {
        cerr << util::err{} << "Could not initialize the batch process.\n"
             << util::err{} << e.what() << "\n";
        return 1;
    } catch (...) {
        cerr << util::err{}
             << "Unexpected error occured during batch processing.\n";
        return 1;
    }

    cerr << util::err{};
    cerr << "One subcommand is required!\n";

    return 1;
}
