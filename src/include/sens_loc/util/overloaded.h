#ifndef OVERLOADED_H_DBSXWJ0H
#define OVERLOADED_H_DBSXWJ0H

#include <type_traits>

namespace sens_loc::util {

template <class T>
struct always_false : std::false_type {};
/// Helper-Class for comfortable visiting of variants.
/// See https://en.cppreference.com/w/cpp/utility/variant/visit.
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
/// Custom template deduction rule for \c overloaded.
/// See https://en.cppreference.com/w/cpp/utility/variant/visit.
template <class... Ts>
overloaded(Ts...)->overloaded<Ts...>;

}  // namespace sens_loc::util

#endif /* end of include guard: OVERLOADED_H_DBSXWJ0H */
