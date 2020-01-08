#ifndef TOOL_MACRO_H_R4SLHAGL
#define TOOL_MACRO_H_R4SLHAGL

/** @file This file defines a macro to reduce the boiler plate to create a new
 * tool. Use \c MAIN_HEAD and \c MAIN_TAIL to wrap the main-function with
 * proper exception handling for the whole program and to enfore consistent
 * error messages on system failure.
 */

#define MAIN_HEAD(TOOL_DESCRIPTION)                                            \
    int main(int argc, char** argv) try {                                      \
        using namespace sens_loc;                                              \
        using namespace sens_loc::apps;                                        \
        using namespace std;                                                   \
        CLI::App app{TOOL_DESCRIPTION};                                        \
        app.add_flag_function("-v,--version", print_version(*argv),            \
                              "Print version and exit");                       \
        do


// clang-format off
#define MAIN_TAIL                                                              \
        while (0); /* End of 'do'-statement from 'MAIN_HEAD' */                \
        throw std::runtime_error{"Reached unexpected part of the program"};    \
    } /* End of actual main-fcuntion */                                        \
    catch (const std::exception& e) {                                          \
        std::cerr << sens_loc::util::err{}                                     \
                  << "Severe problem occured while system-setup.\n"            \
                  << "Message:" << e.what() << "\n";                           \
        return 1;                                                              \
    }                                                                          \
    catch (...) {                                                              \
        std::cerr << sens_loc::util::err{}                                     \
                  << "Severe problem occured while system-setup.\n";           \
        return 1;                                                              \
    }
// clang-format on

#endif /* end of include guard: TOOL_MACRO_H_R4SLHAGL */
