#ifndef INTRINSIC_H_LKU5JQYM
#define INTRINSIC_H_LKU5JQYM

#include <sens_loc/camera_models/pinhole.h>

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

#endif /* end of include guard: INTRINSIC_H_LKU5JQYM */
