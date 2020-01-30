#ifndef BATCH_VISITOR_H_WIWKQDOS
#define BATCH_VISITOR_H_WIWKQDOS

#include <system_error>
#include <taskflow/taskflow.hpp>
#include <type_traits>

namespace sens_loc::apps {

/// General purpose function to execute a specific visitor in parallel
/// in order to determine some statistical insight.
/// \tparam Functor Apply this functor for each index.
/// \param start,end inclusive range of integers for the files to be accessed.
/// \param f functor that is applied for each index
template <typename Functor>
Functor parallel_visitation(int start, int end, Functor&& f) noexcept {
    static_assert(std::is_nothrow_invocable_r_v<void, Functor, int>,
                  "Functor needs to be noexcept callable with an 'int' and "
                  "return nothing!");

    if (start > end)
        std::swap(start, end);

    tf::Executor executor(1);
    tf::Taskflow tf;
    tf.parallel_for(start, end + 1, 1, std::forward<Functor>(f));
    executor.run(tf).wait();

    return std::forward<Functor>(f);
}

}  // namespace sens_loc::apps

#endif /* end of include guard: BATCH_VISITOR_H_WIWKQDOS */
