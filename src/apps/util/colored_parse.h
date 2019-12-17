#ifndef EXIT_HANDLER_H_DOZMG48U
#define EXIT_HANDLER_H_DOZMG48U

#define COLORED_APP_PARSE(app, argc, argv)                                     \
    try {                                                                      \
        (app).parse((argc), (argv));                                           \
    } catch (const CLI::ParseError& e) {                                       \
        std::cout << (e.get_exit_code() == 0 ? rang::fg::blue                  \
                                             : rang::fg::red);                 \
        return app.exit(e);                                                    \
    }

#endif /* end of include guard: EXIT_HANDLER_H_DOZMG48U */
