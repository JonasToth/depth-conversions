#include "converters.h"

#include <CLI/CLI.hpp>
#include <fstream>
#include <gsl/gsl>
#include <iostream>
#include <memory>
#include <rang.hpp>
#include <sens_loc/io/intrinsics.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <sens_loc/version.h>
#include <stdexcept>
#include <string>
#include <variant>

namespace detail {

/// Helper type for the visitor to create the proper converter.
/// Necessary to create a dependent type.
template <class T>
struct always_false : std::false_type {};

using intrinsic_variant =
    std::variant<sens_loc::camera_models::pinhole<double>,
                 sens_loc::camera_models::equirectangular<double>>;

/// This function is a little monster for template reasons.
/// In order to instantiate all necessary batch conversions
/// (feature-type, like bearing_angle) combined with
/// camera_models (e.g. the pinhole model). As everything is
/// templated instead of virtually dispatched these
/// instantiations need to be explicitly written down.
template <template <typename> typename Converter, typename... Arguments>
std::unique_ptr<sens_loc::apps::batch_converter>
make_converter(const sens_loc::apps::file_patterns& files,
               sens_loc::apps::depth_type           t,
               const intrinsic_variant&             intrinsic,
               Arguments&&... args) {
    using namespace sens_loc::camera_models;
    using sens_loc::apps::batch_converter;

    return std::visit(
        [&](auto&& arg) -> std::unique_ptr<batch_converter> {
            using Intrinsic = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<Intrinsic, pinhole<double>>) {
                static_assert(
                    std::is_base_of_v<batch_converter,
                                      Converter<pinhole<double>>>,
                    "Converter must derive from apps::batch_converter");

                return std::make_unique<Converter<pinhole<double>>>(
                    files, t, arg, std::forward<Arguments>(args)...);
                // NOLINTNEXTLINE(readability-misleading-indentation)
            } else if constexpr (std::is_same_v<Intrinsic,
                                                equirectangular<double>>) {
                static_assert(
                    std::is_base_of_v<batch_converter,
                                      Converter<equirectangular<double>>>,
                    "Converter must derive from apps::batch_converter");
                return std::make_unique<Converter<equirectangular<double>>>(
                    files, t, arg, std::forward<Arguments>(args)...);
            } else
                static_assert(always_false<Intrinsic>::value,
                              "not all models were considered");
        },
        intrinsic);
}
}  // namespace detail

/// \defgroup conversion-driver depth-image converter
///
/// All code that is written to use the library and implement a program
/// that converts depth-maps/laser-scan into derived image types.

/// Parallelized driver for depth-image conversion-tool
/// \sa sens_loc::conversion
/// \ingroup conversion-driver
/// \returns 0 if all images could be converted, 1 if any image fails
int main(int argc, char** argv) try {
    using namespace sens_loc;
    using namespace std;

    CLI::App app{
        "Batch-conversion of depth images to various derived image-types."};
    app.require_subcommand(1);
    app.fallthrough();
    app.footer("\n\n"
               "An example invocation of the tool is:\n"
               "\n"
               "depth2x bearing --calibration intrinsic.txt \\\n"
               "                --input depth_{:04d}.png \\\n"
               "                --start 0 \\\n"
               "                --end 100 \\\n"
               "                --horizontal horizontal_{:04d}.png\n"
               "\n"
               "This will read 'depth_0000.png ...' and create "
               "'horizontal_0000.png ...' \n"
               "in the working directory");

    gsl::span<gsl::zstring<>> arguments(argv, argc);

    auto print_version = [arguments](int /*count*/) {
        cout << arguments.at(0) << " " << get_version() << "\n";
        exit(0);
    };
    app.add_flag_function("-v,--version", print_version,
                          "Print version and exit");

    string calibration_file;
    app.add_option("-c,--calibration", calibration_file,
                   "File that contains calibration parameters for the camera")
        ->check(CLI::ExistingFile);

    string camera_model = "pinhole";
    app.add_set("-m,--model", camera_model, {"pinhole", "equirectangular"},
                "Camera model that describes the project of the pixels "
                "into cartesian space. Must match with the '--calibration'"
                " file.",
                /*defaulted=*/true);

    apps::file_patterns files;
    app.add_option("-i,--input", files.input,
                   "Input pattern for image, e.g. \"depth-{}.png\"")
        ->required();

    string input_type = "pinhole-depth";
    app.add_set("-t,--type", input_type, {"pinhole-depth", "pinhole-range"},
                "Type of input depth images, either euclidean depths "
                "(pinhole-range) or orthographic depths (pinhole-depth)",
                /*defaulted=*/true);

    int start_idx;
    app.add_option("-s,--start", start_idx, "Start index of batch, inclusive")
        ->required();
    int end_idx;
    app.add_option("-e,--end", end_idx, "End index of batch, inclusive")
        ->required();

    // Bearing angle images territory
    CLI::App* bearing_cmd = app.add_subcommand(
        "bearing", "Convert depth images into bearing angle images");
    bearing_cmd->footer("\n\n"
                        "An example invocation of the tool is:\n"
                        "\n"
                        "depth2x bearing --calibration intrinsic.txt \\\n"
                        "                --input depth_{:04d}.png \\\n"
                        "                --start 0 \\\n"
                        "                --end 100 \\\n"
                        "                --horizontal horizontal_{:04d}.png\n"
                        "                --vertical vertical_{:04d}.png\n"
                        "\n"
                        "This will read 'depth_0000.png ...' and create "
                        "'horizontal_0000.png vertical_0000.png ...' \n"
                        "in the working directory");

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
    CLI::App* range_cmd = app.add_subcommand(
        "range", "Convert depth images into range images (laser-scan like)");
    range_cmd->footer("\n\n"
                      "An example invocation of the tool is:\n"
                      "\n"
                      "depth2x range --calibration intrinsic.txt \\\n"
                      "              --input depth_{:04d}.png \\\n"
                      "              --start 0 \\\n"
                      "              --end 100 \\\n"
                      "              --output range_{:04d}.png"
                      "\n"
                      "This will read 'depth_0000.png ...' and create "
                      "'range_0000.png ...' in the working directory.");
    range_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the range images.")
        ->required();

    // curvature images territory
    double upper_bound = +20.;  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    double lower_bound = -20.;  // NOLINT(cppcoreguidelines-avoid-magic-numbers)

    CLI::App* mean_curv_cmd = app.add_subcommand(
        "mean-curvature", "Convert depth images into mean-curvature images");
    mean_curv_cmd->footer(
        "\n\n"
        "An example invocation of the tool is:\n"
        "\n"
        "depth2x mean-curvature --calibration intrinsic.txt \\\n"
        "                       --input depth_{:04d}.png \\\n"
        "                       --start 0 \\\n"
        "                       --end 100 \\\n"
        "                       --output mean_{:04d}.png \\\n"
        "                       --lower-bound -20.0 \\\n"
        "                       --upper-bound 20.0"
        "\n"
        "This will read 'depth_0000.png ...' and create "
        "'mean_0000.png ...' in the working directory.");
    mean_curv_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the mean-curvature images.")
        ->required();
    mean_curv_cmd->add_option(
        "-u,--upper-bound", upper_bound,
        "Define an upper bound that curvature values are clamped to.",
        /*defaulted=*/true);
    mean_curv_cmd->add_option(
        "-l,--lower-bound", lower_bound,
        "Define an lower bound that curvature values are clamped to.",
        /*defaulted=*/true);

    CLI::App* gauss_curv_cmd = app.add_subcommand(
        "gauss-curvature",
        "Convert depth images into gaussian-curvature images");
    gauss_curv_cmd->footer(
        "\n\n"
        "An example invocation of the tool is:\n"
        "\n"
        "depth2x gauss-curvature --calibration intrinsic.txt \\\n"
        "                        --input depth_{:04d}.png \\\n"
        "                        --start 0 \\\n"
        "                        --end 100 \\\n"
        "                        --output gauss_{:04d}.png \\\n"
        "                        --lower-bound -1.0 \\\n"
        "                        --upper-bound 1.0"
        "\n"
        "This will read 'depth_0000.png ...' and create "
        "'gauss_0000.png ...' in the working directory.");
    gauss_curv_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the gaussian-curvature images.")
        ->required();
    gauss_curv_cmd->add_option(
        "-u,--upper-bound", upper_bound,
        "Define an upper bound that curvature values are clamped to.",
        /*defaulted=*/true);
    gauss_curv_cmd->add_option(
        "-l,--lower-bound", lower_bound,
        "Define an lower bound that curvature values are clamped to.",
        /*defaulted=*/true);

    // Max-Curve images
    CLI::App* max_curve_cmd = app.add_subcommand(
        "max-curve", "Convert depth images into max-curve images");
    max_curve_cmd->footer("\n\n"
                          "An example invocation of the tool is:\n"
                          "\n"
                          "depth2x max-curve --calibration intrinsic.txt \\\n"
                          "                  --input depth_{:04d}.png \\\n"
                          "                  --start 0 \\\n"
                          "                  --end 100 \\\n"
                          "                  --output max_curve_{:04d}.png"
                          "\n"
                          "This will read 'depth_0000.png ...' and create "
                          "'max_curve_0000.png ...' in the working directory.");
    max_curve_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the max-curve images.")
        ->required();

    // Flexion images
    CLI::App* flexion_cmd = app.add_subcommand(
        "flexion", "Convert depth images into flexion images");
    flexion_cmd->footer("\n\n"
                        "An example invocation of the tool is:\n"
                        "\n"
                        "depth2x flexion --calibration intrinsic.txt \\\n"
                        "                --input depth_{:04d}.png \\\n"
                        "                --start 0 \\\n"
                        "                --end 100 \\\n"
                        "                --output flexion_{:04d}.png"
                        "\n"
                        "This will read 'depth_0000.png ...' and create "
                        "'flexion_0000.png ...' in the working directory.");
    flexion_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the flexion images.")
        ->required();

    // Flexion images
    CLI::App* scale_cmd = app.add_subcommand(
        "scale", "Scale depth images and add an optional offset.");
    scale_cmd->footer("\n\n"
                      "An example invocation of the tool is:\n"
                      "\n"
                      "depth2x scale --input depth_{:04d}.png \\\n"
                      "              --start 0 \\\n"
                      "              --end 100 \\\n"
                      "              --output scale_{:04d}.png \\\n"
                      "              --factor 8.0"
                      "\n"
                      "This will read 'depth_0000.png ...' and create "
                      "'scale_0000.png ...' in the working directory.");
    scale_cmd
        ->add_option("-o,--output", files.output,
                     "Output pattern for the scaled images.")
        ->required();
    double scale_factor = 1.;
    scale_cmd->add_option("-f,--factor", scale_factor,
                          "Real number that is multipled to every depth value.",
                          /*defaulted=*/true);
    double scale_delta = 0.;
    scale_cmd->add_option("-d,--delta", scale_delta,
                          "Real number that is added to every depth value.",
                          /*defaulted=*/true);

    CLI11_PARSE(app, argc, argv);

    // Options that are always required are checked first.
    ifstream cali_fstream{calibration_file};

    const auto potential_intrinsic =
        [&]() -> optional<detail::intrinsic_variant> {
        using camera_models::equirectangular;
        using camera_models::pinhole;
        // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOAD_INTRINSIC(model_name)                                             \
    if (camera_model == #model_name) {                                         \
        auto r = io<double, model_name>::load_intrinsic(cali_fstream);         \
        if (r)                                                                 \
            return *r;                                                         \
        return nullopt;                                                        \
    }
        LOAD_INTRINSIC(pinhole);
        LOAD_INTRINSIC(equirectangular);

#undef LOAD_INTRINSIC

        UNREACHABLE("unexpected camera model received "  // LCOV_EXCL_LINE
                    "from command line parsing");        // LCOV_EXCL_LINE
    }();

    // FIXME: Not nice, but scale_cmd is the only command that does not
    // require the intrinsic. Consequently if it is not given, some other
    // command is expected. This error will then make sense.
    if (!potential_intrinsic && !(*scale_cmd)) {
        cerr << util::err{};
        cerr << "Could not load intrinsic calibration \"" << rang::style::bold
             << calibration_file << rang::style::reset << "\"!\n";
        return 1;
    }

    try {
        Expects(potential_intrinsic);

        auto c = [&]() -> unique_ptr<apps::batch_converter> {
            using namespace apps;
            auto input_enum = apps::str_to_depth_type(input_type);
            if (*scale_cmd)
                return make_unique<apps::scale_converter>(
                    files, input_enum, scale_factor, scale_delta);

            if (*bearing_cmd)
                return detail::make_converter<bearing_converter>(
                    files, input_enum, *potential_intrinsic);
            if (*flexion_cmd)
                return detail::make_converter<flexion_converter>(
                    files, input_enum, *potential_intrinsic);
            if (*gauss_curv_cmd)
                return detail::make_converter<gauss_curv_converter>(
                    files, input_enum, *potential_intrinsic, lower_bound,
                    upper_bound);
            if (*max_curve_cmd)
                return detail::make_converter<max_curve_converter>(
                    files, input_enum, *potential_intrinsic);
            if (*mean_curv_cmd)
                return detail::make_converter<mean_curv_converter>(
                    files, input_enum, *potential_intrinsic, lower_bound,
                    upper_bound);
            if (*range_cmd)
                return detail::make_converter<range_converter>(
                    files, input_enum, *potential_intrinsic);

            throw std::invalid_argument{"target type for conversion required!"};
        }();
        return c->process_batch(start_idx, end_idx) ? 0 : 1;
    } catch (const std::invalid_argument& e) {
        cerr << util::err{} << "Could not initialize the batch process.\n"
             << util::err{} << e.what() << "\n";
        return 1;
    } catch (...) {
        cerr << util::err{}
             << "Unexpected error occured during batch processing.\n";
        return 1;
    }

    UNREACHABLE("All possible options should have terminated already");
} catch (const std::exception& e) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n"
              << "Message:" << e.what();
    return 1;
} catch (...) {
    std::cerr << sens_loc::util::err{}
              << "Severe problem occured while system-setup.\n";
    return 1;
}
