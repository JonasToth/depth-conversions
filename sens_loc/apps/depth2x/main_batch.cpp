#include "batch_converter.h"

#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <rang.hpp>
#include <sens_loc/conversion/depth_to_bearing.h>
#include <sens_loc/conversion/depth_to_curvature.h>
#include <sens_loc/conversion/depth_to_flexion.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/conversion/depth_to_max_curve.h>
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
            throw std::invalid_argument{
                "Missing output pattern for at least one bearing direction"};
        }
    }

    virtual ~bearing_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.horizontal.empty() || !_files.vertical.empty() ||
                !_files.diagonal.empty() || !_files.antidiagonal.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

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
        check_output_exists(files);
    }

    virtual ~range_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.output.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

        cv::Mat depth_converted(depth_image.rows, depth_image.cols,
                                depth_image.type());
        euclid_depth.convertTo(depth_converted, depth_image.type());

        bool success =
            cv::imwrite(fmt::format(_files.output, idx), depth_converted);

        return success;
    }

    const camera_models::pinhole &intrinsic;
};

class gauss_curv_converter : public batch_converter {
  public:
    gauss_curv_converter(const file_patterns &         files,
                         const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {
        check_output_exists(files);
    }
    virtual ~gauss_curv_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.output.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

        const cv::Mat gauss = depth_to_gaussian_curvature<double, double>(
            euclid_depth, intrinsic);

        bool success = cv::imwrite(fmt::format(_files.output, idx), gauss);

        return success;
    }

    const camera_models::pinhole &intrinsic;
};

class mean_curv_converter : public batch_converter {
  public:
    mean_curv_converter(const file_patterns &         files,
                        const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {
        check_output_exists(files);
    }
    virtual ~mean_curv_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.output.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

        const cv::Mat mean =
            depth_to_mean_curvature<double, double>(euclid_depth, intrinsic);

        bool success = cv::imwrite(fmt::format(_files.output, idx), mean);

        return success;
    }

    const camera_models::pinhole &intrinsic;
};

class max_curve_converter : public batch_converter {
  public:
    max_curve_converter(const file_patterns &         files,
                        const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {
        check_output_exists(files);
    }
    virtual ~max_curve_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.output.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

        const cv::Mat max_curve =
            depth_to_max_curve<double, double>(euclid_depth, intrinsic);
        const cv::Mat curve_ushort =
            convert_max_curve<double, ushort>(max_curve);

        bool success =
            cv::imwrite(fmt::format(_files.output, idx), curve_ushort);

        return success;
    }

    const camera_models::pinhole &intrinsic;
};

class flexion_converter : public batch_converter {
  public:
    flexion_converter(const file_patterns &         files,
                      const camera_models::pinhole &intrinsic)
        : batch_converter(files)
        , intrinsic{intrinsic} {
        check_output_exists(files);
    }
    virtual ~flexion_converter() = default;

  private:
    bool process_file(const cv::Mat &depth_image, int idx) const
        noexcept override {
        Expects(!_files.output.empty());

        using namespace conversion;

        const cv::Mat euclid_depth =
            depth_to_laserscan<double, ushort>(depth_image, intrinsic);

        const cv::Mat flexion =
            depth_to_flexion<double, double>(euclid_depth, intrinsic);
        const cv::Mat flexion_ushort = convert_flexion<double, ushort>(flexion);

        bool success =
            cv::imwrite(fmt::format(_files.output, idx), flexion_ushort);

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
