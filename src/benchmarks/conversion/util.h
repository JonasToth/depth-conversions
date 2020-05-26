#include <sens_loc/camera_models/equirectangular.h>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <sens_loc/math/angle_conversion.h>
#include <sens_loc/math/image.h>
#include <stdexcept>
#include <tuple>

inline std::tuple<sens_loc::math::image<ushort>,
                  sens_loc::math::image<float>,
                  sens_loc::camera_models::pinhole<float>>
get_data() {
    const sens_loc::camera_models::pinhole<float> p = {
        /*w=*/960,       /*h=*/540,      /*fx=*/519.226F,
        /*fy=*/479.462F, /*cx=*/522.23F, /*cy=*/272.737F,
    };
    const std::optional<sens_loc::math::image<ushort>> img =
        sens_loc::io::load_image<ushort>("conversion/data0-depth.png",
                                         cv::IMREAD_UNCHANGED);
    if (!img)
        throw std::runtime_error{"No Data Found!"};
    const auto euclid = sens_loc::conversion::depth_to_laserscan(*img, p);

    return std::make_tuple(*img, euclid, p);
}


inline std::tuple<sens_loc::math::image<float>,
                  sens_loc::camera_models::equirectangular<float>>
get_data_laserscan() {
    using namespace sens_loc;
    const camera_models::equirectangular<float> e_float{
        /*width=*/3600,
        /*height=*/800,
        /*theta_range=*/
        {math::deg_to_rad(50.F), math::deg_to_rad(130.F)}};

    const std::optional<math::image<ushort>> img = io::load_image<ushort>(
        "conversion/laserscan-depth.png", cv::IMREAD_UNCHANGED);
    if (!img)
        throw std::runtime_error{"No Data Found!"};

    const auto euclid = math::convert<float>(*img);

    return std::make_tuple(euclid, e_float);
}
