#ifndef EIGEN_TYPES_H_HYDVW3MJ
#define EIGEN_TYPES_H_HYDVW3MJ

#include <Eigen/Eigen>

namespace sens_loc::math {
/// This typedef is very similar to Eigen3 own typedeffing, but lets
/// the Scalar type a template parameter.
template <typename Real, int Dim = Eigen::Dynamic>
using vec = Eigen::Matrix<Real, Dim, 1>;
}  // namespace sens_loc::math

#endif /* end of include guard: EIGEN_TYPES_H_HYDVW3MJ */
