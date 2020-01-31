#ifndef CONCEPTS_H_DYRKVMES
#define CONCEPTS_H_DYRKVMES

#include <sens_loc/math/coordinate.h>
#include <type_traits>

namespace sens_loc::camera_models {

/// This type_trait checks the requirements on a type that implements
/// a camera model. These requirements need to be fullfilled in order to
/// be usable as a type for the conversion functions.
///
/// \sa camera_models::pinhole
/// \sa camera_models::equirectangular
///
/// - the intrinsic must have its precisions of calculation templated (either
///   float or double must be accepted)
/// - the intrinsic must provide access to its internal precision via
///   \c Intrinsic::real_type
/// - expose the valid width with \c w() and height with \c h()
/// - the intrinsic must be able to backproject a pixel point to the unit sphere
///   both with pixel and subpixel precision
/// - it is __NOT__ necessary to provide a forward projection of
///   camera coordinates into the image
/// - the backprojected point needs to be unitarian, which can not be enforced
///   with concepts alone, but requires contracts on the type \c sphere_coord
///
/// The code that needs to work:
/// \code
/// Intrinsic<float> i; // Create an intrinsic with floating point precision
/// Expects(i.w() > i.h()); // use width and height
/// auto p1 = i.pixel_to_sphere({42, 42});
/// auto p2 = i.pixel_to_sphere({42.5f, 42.5f});
/// Expects(p1.norm() == 1.0f && p2.norm() == 1.0f);
/// \endcode
template <template <typename> typename Intrinsic, typename Real>
inline constexpr bool is_intrinsic_v =
    // clang-format off
    std::is_floating_point_v<Real>
    && 
    std::is_same_v<typename Intrinsic<Real>::real_type, Real>
    &&
    std::is_invocable_r_v<int,
                          decltype(&Intrinsic<Real>::w),
                          Intrinsic<Real>>
    &&
    std::is_invocable_r_v<int,
                          decltype(&Intrinsic<Real>::h),
                          Intrinsic<Real>>
    && 
    std::is_invocable_r_v<math::sphere_coord<Real>,
                          decltype(&Intrinsic<Real>::template pixel_to_sphere<int>),
                          Intrinsic<Real>,
                          math::pixel_coord<int>>
    &&
    std::is_invocable_r_v<math::sphere_coord<Real>,
                          decltype(&Intrinsic<Real>::template pixel_to_sphere<Real>),
                          Intrinsic<Real>,
                          math::pixel_coord<Real>>
    &&
    std::is_invocable_r_v<math::pixel_coord<Real>,
                          decltype(&Intrinsic<Real>::template camera_to_pixel<Real>),
                          Intrinsic<Real>,
                          math::camera_coord<Real>>
    &&
    std::is_invocable_r_v<math::pixel_coord<int>,
                          decltype(&Intrinsic<Real>::template camera_to_pixel<int>),
                          Intrinsic<Real>,
                          math::camera_coord<Real>>;
// clang-format on

}  // namespace sens_loc::camera_models

#endif /* end of include guard: CONCEPTS_H_DYRKVMES */
