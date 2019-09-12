#include <CLI/CLI.hpp>
#include <chrono>
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
#include <string_view>
#include <taskflow/taskflow.hpp>
#include <vector>

using namespace sens_loc;
using namespace std;

namespace bearing {
struct file_patterns {
    string_view input;
    string_view horizontal;
    string_view vertical;
    string_view diagonal;
    string_view antidiagonal;
};

/// This file substitues 'index' into the patterns (if existent) and calculates
/// bearing angle images for it.
/// If any operation fails 'false' is returned. On success 'true' is returned.
bool process_file(const file_patterns &         p,
                  const camera_models::pinhole &intrinsic, int index) {
    Expects(!p.input.empty());
    Expects(!p.horizontal.empty() || !p.vertical.empty() ||
            !p.diagonal.empty() || !p.antidiagonal.empty());

    using namespace conversion;

    const std::string input_file = fmt::format(p.input, index);
    optional<cv::Mat> depth_image =
        io::load_image(input_file, cv::IMREAD_UNCHANGED);

    if (!depth_image)
        return false;

    const cv::Mat euclid_depth =
        depth_to_laserscan<double, ushort>(*depth_image, intrinsic);

    bool final_result = true;
#define BEARING_PROCESS(DIRECTION)                                             \
    if (!p.DIRECTION.empty()) {                                                \
        cv::Mat bearing =                                                      \
            depth_to_bearing<direction::DIRECTION, double, double>(            \
                euclid_depth, intrinsic);                                      \
        bool success = cv::imwrite(fmt::format(p.DIRECTION, index),            \
                                   convert_bearing<double, ushort>(bearing));  \
        if (!success)                                                          \
            final_result = false;                                              \
    }

    BEARING_PROCESS(horizontal)
    BEARING_PROCESS(vertical)
    BEARING_PROCESS(diagonal)
    BEARING_PROCESS(antidiagonal)

#undef BEARING_PROCESS

    return true;
}

}  // namespace bearing

int main(int argc, char **argv) {
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

    string input_file;
    app.add_option("-i,--input", input_file,
                   "Input pattern for image, e.g. \"depth-{}.png\"")
        ->required();

    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index of batch, inclusive")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index of batch, inclusive")
        ->required();

    // The following options only apply to bearing angle images.
    string bearing_hor_name;
    app.add_option(
        "--horizontal", bearing_hor_name,
        "Calculate horizontal bearing angle image and write to this pattern");
    string bearing_ver_name;
    app.add_option(
        "--vertical", bearing_ver_name,
        "Calculate vertical bearing angle and write to this pattern");
    string bearing_dia_name;
    app.add_option(
        "--diagonal", bearing_dia_name,
        "Calculate diagonal bearing angle and write to this pattern");
    string bearing_ant_name;
    app.add_option(
        "--anti-diagonal", bearing_ant_name,
        "Calculate anti-diagonal bearing angle and write to this pattern");

    CLI11_PARSE(app, argc, argv);

    if (bearing_hor_name.empty() && bearing_ver_name.empty() &&
        bearing_dia_name.empty() && bearing_ant_name.empty()) {
        cerr << util::err{};
        cerr << "At least one output pattern required!\n";
        return 1;
    }


    ifstream                         calibration_fstream{calibration_file};
    optional<camera_models::pinhole> intrinsic =
        io::load_pinhole_intrinsic(calibration_fstream);

    if (!intrinsic) {
        cerr << util::err{};
        cerr << "Could not load intrinsic calibration \"" << rang::style::bold
             << calibration_file << rang::style::reset << "\"!\n";
        return 1;
    }
    bearing::file_patterns files{
        .input        = input_file,
        .horizontal   = bearing_hor_name,
        .vertical     = bearing_ver_name,
        .diagonal     = bearing_dia_name,
        .antidiagonal = bearing_ant_name,
    };

    int fails       = 0;
    int return_code = 0;
    {
        tf::Executor executor;
        tf::Taskflow tf;
        mutex        cout_mutex;

        tf.parallel_for(
            start_idx, end_idx + 1, 1,
            [&files, &cout_mutex, &intrinsic, &return_code, &fails](int idx) {
                const bool success =
                    bearing::process_file(files, *intrinsic, idx);
                if (!success) {
                    lock_guard l(cout_mutex);
                    fails++;
                    cerr << util::err{};
                    cerr << "Could not process index \"" << rang::style::bold
                         << idx << "\"" << rang::style::reset << "!\n";
                    return_code = 1;
                }
            });

        const auto before = chrono::steady_clock::now();
        executor.run(tf).wait();
        const auto after = chrono::steady_clock::now();

        cerr << util::info{};
        cerr << "Processing " << rang::style::bold
             << end_idx - start_idx + 1 - fails << rang::style::reset
             << " images took " << rang::style::bold
             << chrono::duration_cast<chrono::seconds>(after - before).count()
             << "" << rang::style::reset << " seconds!\n";

        if (fails > 0)
            cerr << util::warn{} << "Encountered " << rang::style::bold << fails
                 << rang::style::reset << " problematic files!\n";
    }

    return return_code;
}
