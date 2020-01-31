#include <sens_loc/math/coordinate.h>

using namespace sens_loc::math;

#if defined(ASSIGN_DIFFERENT_REAL_TYPE)
static void fail_assignment() {
    pixel_coord<int>    p1;
    pixel_coord<double> p2;

    p1 = p2;
}
#endif

#if defined(ASSIGN_DIFFERENT_SYSTEM)
static void fail_assignment() {
    image_coord<double> p1;
    pixel_coord<double> p2;

    p1 = p2;
}
#endif

#if defined(ACCESS_CONVENTION)
static void fail_assignment() {
    image_coord<double> p1;
    p1.u();
}
#endif

#if defined(ACCESS_CROSS_PRODUCT_2D)
static void fail_assignment() {
    image_coord<double> p1;
    image_coord<double> p2;
    (void) p1.cross(p2);
}
#endif

#if defined(OPERATOR_SCALING_PIXEL)
static void fail_scale_pixel() {
    pixel_coord<double> p0;
    (void) 20. * p0;
}
#endif

#if defined(OPERATOR_SCALING_IMAGE)
static void fail_scale_pixel() {
    image_coord<double> p0;
    (void) 20. * p0;
}
#endif
#if defined(OPERATOR_SCALING_WORLD)
static void fail_scale_pixel() {
    world_coord<double> p0;
    (void) 20. * p0;
}
#endif
#if defined(SPHERICAL_SCALING_TO_SPHERICAL)
static void fail_scale_pixel() {
    sphere_coord<double> p0;
    sphere_coord         res = 20. * p0;
}
#endif

int main() {
    return 0;
}
