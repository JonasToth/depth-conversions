#ifndef PINHOLE_H_I3RZULX9
#define PINHOLE_H_I3RZULX9

#include <cmath>

namespace sens_loc { namespace camera_models {

/// This struct is a container holding parameters for the pinhole camera model.
struct pinhole_parameters {
    int    w;   //< width of the image
    int    h;   //< height of the image
    double fx;  //< x-coordinate of focal length
    double fy;  //< y-corrdiante of focal length
    double cx;  //< x-coordinate of image center
    double cy;  //< y-coordinate of image center

    double p1 = 0.0; //< first order tangential distortion coefficient
    double p2 = 0.0; //< second order tangential distortion coefficient

    double k1 = 0.0; //< first order radial distortion coefficient
    double k2 = 0.0; //< second order radial distortion coefficient
    double k3 = 0.0; //< third order radial distortion coefficient

    /// Calculates the angular resolution of one pixel in x-direction.
    /// \returns radians/pixel in x direction
    double x_resolution() const noexcept;
    /// Calculates the angular resolution of one pixel in y-direction.
    /// \returns radians/pixel in y direction
    double y_resolution() const noexcept;
};

inline double pinhole_parameters::x_resolution() const noexcept {
    return std::atan(cx / fx) / cx;
}
inline double pinhole_parameters::y_resolution() const noexcept {
    return std::atan(cy / fy) / cy;
}
}}  // namespace sens_loc::camera_models


#endif /* end of include guard: PINHOLE_H_I3RZULX9 */
