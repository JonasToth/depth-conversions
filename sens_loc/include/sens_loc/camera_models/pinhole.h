#ifndef PINHOLE_H_I3RZULX9
#define PINHOLE_H_I3RZULX9

#include <cmath>
#include <gsl/gsl>
#include <sens_loc/math/constants.h>
#include <utility>

namespace sens_loc { namespace camera_models {

/// This struct is a container holding parameters for the pinhole camera model.
struct pinhole {
    int    w  = 0;    //< width of the image
    int    h  = 0;    //< height of the image
    double fx = 0.0;  //< x-coordinate of focal length
    double fy = 0.0;  //< y-corrdiante of focal length
    double cx = 0.0;  //< x-coordinate of image center
    double cy = 0.0;  //< y-coordinate of image center

    double p1 = 0.0;  //< first order tangential distortion coefficient
    double p2 = 0.0;  //< second order tangential distortion coefficient

    double k1 = 0.0;  //< first order radial distortion coefficient
    double k2 = 0.0;  //< second order radial distortion coefficient
    double k3 = 0.0;  //< third order radial distortion coefficient

    /// This method calculates the angle (rad) of the rays between two pixels.
    /// \returns radians
    [[nodiscard]] double phi(int u0, int v0, int u1, int v1) const noexcept;

    [[nodiscard]] std::tuple<double, double, double>
    project_to_sphere(int u, int v) const noexcept;
};

inline double pinhole::phi(int u0, int v0, int u1, int v1) const noexcept {
    Expects(u0 >= 0);
    Expects(v0 >= 0);
    Expects(u1 >= 0);
    Expects(v1 >= 0);

    const auto [xs0, ys0, zs0] = project_to_sphere(u0, v0);
    const auto [xs1, ys1, zs1] = project_to_sphere(u1, v1);
    const auto cos_phi         = xs0 * xs1 + ys0 * ys1 + zs0 * zs1;

    Ensures(cos_phi > -1.);
    Ensures(cos_phi < +1.);

    const auto angle = std::acos(cos_phi);
    return angle;
}

inline std::tuple<double, double, double>
pinhole::project_to_sphere(int u, int v) const noexcept {
    Expects(fy > 0.);
    Expects(cy > 0.);
    Expects(p1 == 0.);
    Expects(p2 == 0.);
    Expects(k1 == 0.);
    Expects(k2 == 0.);
    Expects(k3 == 0.);

    const double x = (double(u) - cx) / fx;
    const double y = (double(v) - cy) / fy;
    const double z = 1.;

    const double factor = std::sqrt(1. + x * x + y * y) / (1. + x * x + y * y);
    const double xs     = factor * x;
    const double ys     = factor * y;
    const double zs     = factor * z;

    Ensures(std::abs(xs * xs + ys * ys + zs * zs - 1.) < 0.00000001);

    return std::make_tuple(xs, ys, zs);
}
}}  // namespace sens_loc::camera_models


#endif /* end of include guard: PINHOLE_H_I3RZULX9 */
