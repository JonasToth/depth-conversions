#include <CLI/CLI.hpp>
#include <cstdlib>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <rang.hpp>
#include <sens_loc/util/console.h>
#include <sens_loc/io/image.h>

using namespace sens_loc;

int main(int argc, char **argv) {
    CLI::App app{"Convert depth images to pointclouds"};

    std::string calibration_file;
    app.add_option("-c,--calibration", calibration_file,
                   "File that contains calibration parameters for the camera")
        ->required()
        ->check(CLI::ExistingFile);

    std::string input_file{"/home/jonas/owncloud/Freiberg/Masterarbeit/"
                           "full_data_sets/lehrpfad/raw_data/data196.jpg"};
    app.add_option("-i,--input", input_file, "Input depth map")
        ->required()
        ->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    std::optional<cv::Mat> image = image::load(input_file);

    if (!image) {
        std::cerr << util::err{};
        std::cerr << "Could not load image \"" << rang::style::bold
                  << input_file << rang::style::reset << "\"!\n";
        std::exit(1);
    }

    return 0;
}
