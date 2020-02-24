#ifndef PROGRESS_BAR_OBSERVER_H_0L8ZR7QT
#define PROGRESS_BAR_OBSERVER_H_0L8ZR7QT

#include "rang.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <gsl/gsl>
#include <iomanip>
#include <sens_loc/util/console.h>
#include <taskflow/core/observer.hpp>

namespace sens_loc::util {

class progress_bar_observer : public tf::ExecutorObserverInterface {
  public:
    constexpr static int max_bars = 50;

    progress_bar_observer(std::int64_t partitions, std::int64_t total_tasks)
        : _partitions{partitions}
        , _task_increment{gsl::narrow_cast<float>(total_tasks) /
                          gsl::narrow_cast<float>(partitions)}
        , _done{0} {
        Expects(total_tasks >= 1);
    }

    progress_bar_observer(const progress_bar_observer&) = delete;
    progress_bar_observer(progress_bar_observer&&)      = delete;
    progress_bar_observer& operator=(const progress_bar_observer&) = delete;
    progress_bar_observer& operator=(progress_bar_observer&&) = delete;
    ~progress_bar_observer() override                         = default;

    void set_up(unsigned /*num_workers*/) override {}
    void on_entry(unsigned /*worker_id*/, tf::TaskView /*task_view*/) override {
        if (!_inital_output) {
            _inital_output = true;
            print_bar();
        }
    }
    void on_exit(unsigned /*worker_id*/, tf::TaskView /*task_view*/) override {
        _done++;
        print_bar();
    }

  private:
    void print_bar() {
        auto  s        = synced();
        float progress = gsl::narrow_cast<float>(_done) /
                         gsl::narrow_cast<float>(_partitions);
        std::cout << "\r" << rang::fg::green << rang::style::bold
                  << std::setw(5)
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
    }

    std::int64_t              _partitions;
    float                     _task_increment;
    std::atomic<std::int64_t> _done;
    std::atomic<bool>         _inital_output = false;
};
}  // namespace sens_loc::util

#endif /* end of include guard: PROGRESS_BAR_OBSERVER_H_0L8ZR7QT */
