#include <CLI/CLI.hpp>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

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

    auto Image = cv::imread(input_file);

    return 0;
}
