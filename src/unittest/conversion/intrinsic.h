#ifndef INTRINSIC_H_LKU5JQYM
#define INTRINSIC_H_LKU5JQYM

#include <sens_loc/camera_models/equirectangular.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/angle_conversion.h>

constexpr sens_loc::camera_models::pinhole<double> p = {
    /*w=*/960,      /*h=*/540,     /*fx=*/519.226,
    /*fy=*/479.462, /*cx=*/522.23, /*cy=*/272.737,
};
constexpr sens_loc::camera_models::pinhole<double> p_double = {
    /*w=*/960,      /*h=*/540,     /*fx=*/519.226,
    /*fy=*/479.462, /*cx=*/522.23, /*cy=*/272.737,
};
constexpr sens_loc::camera_models::pinhole<float> p_float = {
    /*w=*/960,       /*h=*/540,      /*fx=*/519.226f,
    /*fy=*/479.462f, /*cx=*/522.23f, /*cy=*/272.737f,
};


using sens_loc::math::deg_to_rad;
constexpr sens_loc::camera_models::equirectangular<double> e_double{
    /*width=*/1799,
    /*height=*/397,
    /*theta_range=*/{deg_to_rad(50.), deg_to_rad(130.)}};

constexpr sens_loc::camera_models::equirectangular<float> e_float{
    /*width=*/1799,
    /*height=*/397,
    /*theta_range=*/{deg_to_rad(50.f), deg_to_rad(130.f)}};

#endif /* end of include guard: INTRINSIC_H_LKU5JQYM */
