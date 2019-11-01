#include "filter_functor.h"

#include <CLI/CLI.hpp>
#include <gsl/gsl>
#include <memory>
#include <rang.hpp>
#include <sens_loc/preprocess/filter.h>
#include <sens_loc/util/console.h>
#include <sens_loc/util/correctness_util.h>
#include <sens_loc/version.h>
#include <stdexcept>
#include <util/batch_converter.h>
#include <vector>

int main(int argc, char** argv) try {
    using namespace sens_loc;
    using namespace std;

    CLI::App app{"Batch-processing tool to filter depth images and maps"};
    app.require_subcommand();  // Expect one or more filter commands
    app.footer("\n\n"
               "An example invocation of the tool is:\n"
               "\n"
               "depth_filter ");

    gsl::span<gsl::zstring<>> arguments(argv, argc);

    auto print_version = [arguments](int /*count*/) {
        cout << arguments.at(0) << " " << get_version() << "\n";
        exit(0);
    };
    app.add_flag_function("-v,--version", print_version,
                          "Print version and exit");

    apps::file_patterns files;
    app.add_option("-i,--input", files.input,
                   "Input pattern for images to filter; e.g. \"depth-{}.png\"")
        ->required();
    app.add_option(
           "-o,--output", files.output,
           "Output pattern for filtered images; e.g. \"filtered-{}.png\"")
        ->required();

    CLI::App* bilateral_cmd = app.add_subcommand(
        "bilateral", "Apply the bilateral filter to the input.");
    double sigma_color = 20.;
    bilateral_cmd->add_option("-c,--sigma-color", sigma_color,
                              "Defines threshold for color similarity.",
                              /*defaulted=*/true);
    int          distance;
    CLI::Option* distance_option = bilateral_cmd->add_option(
        "-d,--distance", distance,
        "One option to define the relevant neighbourhood, diameter in pixel");
    double       sigma_space;
    CLI::Option* sigma_space_option =
        bilateral_cmd->add_option("-s,--sigma-space", sigma_space,
                                  "Other option to define proximity relation, "
                                  "closeness is computed from that value");

    CLI11_PARSE(app, argc, argv);

    if (distance_option->empty() && sigma_space_option->empty()) {
        cerr << util::err{}
             << "Provide a proximity-measure for the bilateral filter!\n";
        return 1;
    }

    // Create a vector of functors (unique_ptr<filter_interface>) that will
    // be executed in order.
    // Each element is created by one subcommand and its parameters.
    vector<unique_ptr<apps::abstract_filter>> commands;

    for (const auto* cmd : app.get_subcommands()) {
        if (cmd == bilateral_cmd) {
            // Got at pixel-distance as argument.
            if (!distance_option->empty())
                commands.push_back(
                    make_unique<apps::bilateral_filter>(sigma_color, distance));

            // Otherwise sigma space must be provided.
            commands.push_back(
                make_unique<apps::bilateral_filter>(sigma_color, sigma_space));
        }
    }

    Ensures(!commands.empty());

    // Batch-processing will then just execute the functors for each image.
    // It needs to take care, that the elements are moved through.
    // The final step is conversion to U16 and writing to disk.

    return 0;

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
