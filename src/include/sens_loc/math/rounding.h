#ifndef ROUNDING_H_ZL4GHC5J
#define ROUNDING_H_ZL4GHC5J

#include <cmath>
#include <gsl/gsl>
#include <type_traits>

namespace sens_loc::math {

/// Round the value \c val to the nth \c digit.
///
/// \tparam Number arithmetic type that represents a number
/// \param val number to be rounded
/// \param digit The decimal digit the number shall be rounded to. Positive
/// values indicate the position behind the comma for rational numbers, a
/// negative number indicates a position before the comma.
/// \pre \c std::is_arithmetic_v<Number>
/// \pre \c val must be a finite number, so no NaN or Inf is allowed.
/// \post the result is rounded to the nth digit
/// \post \f$digit == 0 => result == val\f$
/// \code
/// roundn(10.055, 2) == 10.06;
/// roundn(10.055, 0) == 10.055;
/// roundn(10.055, -1) == 10.00;
/// roundn(1052.4, -2) == 1100.0;
/// roundn(1052, -2) == 1100;
/// roundn(1052, 2) == 1052;
/// \endcode
template <typename Number>
Number roundn(Number val, int digit) noexcept {
    Expects(std::isfinite(val));
    static_assert(std::is_arithmetic_v<Number>);

    if (digit == 0)
        return val;

    if constexpr (std::is_floating_point_v<Number>) {
        Number potence_10 = std::pow(Number(10), digit);
        return std::round(val * potence_10) / potence_10;
    } else {
        double potence_10 = std::pow(10., digit);
        double rounded    = std::round(val * potence_10) / potence_10;
        return Number(std::round(rounded));
    }
}

}  // namespace sens_loc::math

#endif /* end of include guard: ROUNDING_H_ZL4GHC5J */
