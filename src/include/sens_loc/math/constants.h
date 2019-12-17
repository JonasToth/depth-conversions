#ifndef CONSTANTS_H_ZHT8MULP
#define CONSTANTS_H_ZHT8MULP

namespace sens_loc::math {

/// Arbitrary precision for \f$\pi\f$ and have it either as \c float or
/// \c double.
template <typename T>
constexpr T pi =  // NOLINT(misc-definitions-in-headers)
    T(3.141'592'653'589'793'238'462);

}  // namespace sens_loc::math

#endif /* end of include guard: CONSTANTS_H_ZHT8MULP */
