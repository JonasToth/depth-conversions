#include <sens_loc/util/progress_bar_observer.h>

namespace sens_loc::util {
void progress_bar_observer::on_entry(unsigned /*worker_id*/,
                                     tf::TaskView /*task_view*/) {
    if (!_inital_output) {
        _inital_output = true;
        print_bar();
    }
}

void progress_bar_observer::on_exit(unsigned /*worker_id*/,
                                    tf::TaskView /*task_view*/) {
    _done++;
    print_bar();
}

void progress_bar_observer::print_bar() noexcept {
    auto  s = synced();
    float progress =
        gsl::narrow_cast<float>(_done) / gsl::narrow_cast<float>(_partitions);
    std::cout << "\r" << rang::fg::green << rang::style::bold << std::setw(5)
              << gsl::narrow_cast<int>(std::round(_done * _task_increment))
              << " ";

    // Progress bar is 30 characters wide.
    int bar_elements =
        gsl::narrow_cast<int>(progress * gsl::narrow_cast<float>(max_bars));
    Ensures(bar_elements <= max_bars);
    Ensures(bar_elements >= 0);
    int empty_elements = max_bars - bar_elements;

    std::cout << rang::style::reset << rang::bg::green << rang::fg::green;
    for (int i = 0; i < bar_elements; ++i) {
        std::cout << "#";
    }
    std::cout << rang::bg::blue << rang::fg::blue;
    for (int i = 0; i < empty_elements; ++i) {
        std::cout << ".";
    }
    std::cout << rang::style::reset << std::flush;

    if (_done == _partitions)
        std::cout << std::endl;
}
}  // namespace sens_loc::util
