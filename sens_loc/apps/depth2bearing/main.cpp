#include <CLI/CLI.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <rang.hpp>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/util/console.h>
#include <sens_loc/version.h>
#include <taskflow/taskflow.hpp>

using namespace sens_loc;
using namespace std;

int main(int argc, char **argv) {
    CLI::App app{"Convert depth images to pointclouds"};

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

    string input_file{"/home/jonas/owncloud/Freiberg/Masterarbeit/"
                      "full_data_sets/lehrpfad/raw_data/data196-depth.png"};
    app.add_option("-i,--input", input_file, "Input depth map")
        ->required()
        ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    tf::Executor executor;
    tf::Taskflow taskflow;

    cv::Mat                depth_image;
    io::pinhole_parameters intrinsic;

    taskflow.emplace([&input_file]() {
        // Load the image unchanged, because depth images are encoded specially.
        optional<cv::Mat> image =
            io::load_image(input_file, cv::IMREAD_UNCHANGED);
        if (!image) {
            cerr << util::err{};
            cerr << "Could not load image \"" << rang::style::bold << input_file
                 << rang::style::reset << "\"!\n";
            exit(1);
        }
        return *image;
    });

    taskflow.emplace([&calibration_file]() {
        ifstream calibration_fstream{calibration_file};

        optional<io::pinhole_parameters> calibration =
            io::load_pinhole_intrinsic(calibration_fstream);
        if (!calibration) {
            cerr << util::err{};
            cerr << "Could not intrinsic calibration \"" << rang::style::bold
                 << calibration_file << rang::style::reset << "\"!\n";
            exit(1);
        }
        return *calibration;
    });

    executor.run(taskflow).wait();

    return 0;
}
