#ifndef SCALING_H_SAL4XICS
#define SCALING_H_SAL4XICS

#include <algorithm>
#include <gsl/gsl>
#include <utility>

namespace sens_loc { namespace math {

/// This class is a small wrapper to clarify the mathematical notation
/// of ranges. It does NOT distinguish between \f$[min, max)\f$,
/// \f$[min, max]\f$, \f$(min, max)\f$.
/// \tparam Real underlying type of the range, integer type for example
/// \warning this struct does not ensure that \p min is always smaller \p max.
template <typename Real>
struct numeric_range {
    Real min;  ///< lower bound of the range
    Real max;  ///< uppoer bound of the range
};

/// This function scales \p value from the range \p source_range to the
/// new range \p target_range.
/// \note \p value is clamped to \p source_range
/// \pre the ranges are well formed (\p min is smaller \p max)
/// \returns the scaled value that is clamped to \p target_range.
template <typename Real>
inline constexpr Real scale(numeric_range<Real> source_range,
                            numeric_range<Real> target_range,
                            Real                value) noexcept {
    Expects(source_range.min < source_range.max);
    Expects(target_range.min < target_range.max);

    const Real clamped = std::clamp(value, source_range.min, source_range.max);

    Ensures(clamped <= source_range.max);
    Ensures(clamped >= source_range.min);

    const Real scaled =
        ((target_range.max - target_range.min) * (value - source_range.min)) /
            (source_range.max - source_range.min) +
        target_range.min;

    const Real target_clamped =
        std::clamp(scaled, target_range.min, target_range.max);

    Ensures(target_clamped <= target_range.max);
    Ensures(target_clamped >= target_range.min);

    return target_clamped;
}
}}  // namespace sens_loc::math

#endif /* end of include guard: SCALING_H_SAL4XICS */
