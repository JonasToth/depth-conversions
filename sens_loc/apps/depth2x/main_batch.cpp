#include "batch_converter.h"

#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <rang.hpp>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/util/console.h>
#include <sens_loc/version.h>
#include <stdexcept>
#include <string>
#include <taskflow/taskflow.hpp>


namespace sens_loc { namespace apps {

class bearing_converter : public batch_converter {
  public:
    bearing_converter(const file_patterns &         files,
                      const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {
        if (files.horizontal.empty() && files.vertical.empty() &&
            files.diagonal.empty() && files.antidiagonal.empty()) {
            std::cerr << util::err{};
            std::cerr << "At least one output pattern required!\n";
            throw std::invalid_argument{"Missing output pattern"};
        }
    }

    virtual ~bearing_converter() = default;

  private:
    bool process_file(int idx) const noexcept override {
        Expects(!_files.input.empty());
        Expects(!_files.horizontal.empty() || !_files.vertical.empty() ||
                !_files.diagonal.empty() || !_files.antidiagonal.empty());

        using namespace conversion;

        const std::string      input_file = fmt::format(_files.input, idx);
        std::optional<cv::Mat> depth_image =
            io::load_image(input_file, cv::IMREAD_UNCHANGED);

        if (!depth_image)
            return false;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(*depth_image, intrinsic);

        bool final_result = true;
#define BEARING_PROCESS(DIRECTION)                                             \
    if (!_files.DIRECTION.empty()) {                                           \
        cv::Mat bearing =                                                      \
            depth_to_bearing<direction::DIRECTION, double, double>(            \
                euclid_depth, intrinsic);                                      \
        bool success = cv::imwrite(fmt::format(_files.DIRECTION, idx),         \
                                   convert_bearing<double, ushort>(bearing));  \
        final_result &= success;                                               \
    }

        BEARING_PROCESS(horizontal)
        BEARING_PROCESS(vertical)
        BEARING_PROCESS(diagonal)
        BEARING_PROCESS(antidiagonal)

#undef BEARING_PROCESS

        return final_result;
    }

    const camera_models::pinhole &intrinsic;
};

class range_converter : public batch_converter {
  public:
    range_converter(const file_patterns &         files,
                    const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {

        if (files.output.empty()) {
            std::cerr << util::err{};
            std::cerr << "output pattern required!\n";
            throw std::invalid_argument{"output pattern required\n"};
        }
    }

    virtual ~range_converter() = default;

  private:
    bool process_file(int idx) const noexcept override {
        Expects(!_files.input.empty());
        Expects(!_files.output.empty());

        using namespace conversion;

        const std::string      input_file = fmt::format(_files.input, idx);
        std::optional<cv::Mat> depth_image =
            io::load_image(input_file, cv::IMREAD_UNCHANGED);

        if (!depth_image)
            return false;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(*depth_image, intrinsic);

        cv::Mat depth_converted((*depth_image).rows, (*depth_image).cols,
                                (*depth_image).type());
        euclid_depth.convertTo(depth_converted, (*depth_image).type());

        bool success =
            cv::imwrite(fmt::format(_files.output, idx), depth_converted);

        return success;
    }

    const camera_models::pinhole &intrinsic;
};

}}  // namespace sens_loc::apps

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

    if (*bearing_cmd) {
        apps::bearing_converter c(files, *intrinsic);
        return c.process_batch(start_idx, end_idx);
    } else if (*range_cmd) {
        apps::range_converter c(files, *intrinsic);
        return c.process_batch(start_idx, end_idx);
    } else {
        cerr << util::err{};
        cerr << "One subcommand is required!\n";
        return 1;
    }
}
