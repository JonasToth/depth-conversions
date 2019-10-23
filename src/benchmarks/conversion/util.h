#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/conversion/depth_to_laserscan.h>
#include <sens_loc/io/image.h>
#include <stdexcept>
#include <tuple>

inline std::tuple<sens_loc::math::image<ushort>, sens_loc::math::image<float>,
                  sens_loc::camera_models::pinhole<float>>
get_data() {
    constexpr sens_loc::camera_models::pinhole<float> p = {
        /*w=*/960,      /*h=*/540,     /*fx=*/519.226,
        /*fy=*/479.462, /*cx=*/522.23, /*cy=*/272.737,
    };
    const std::optional<sens_loc::math::image<ushort>> img =
        sens_loc::io::load_image<ushort>("conversion/data0-depth.png",
                                         cv::IMREAD_UNCHANGED);
    if (!img)
        throw std::runtime_error{"No Data Found!"};
    const auto euclid = sens_loc::conversion::depth_to_laserscan(*img, p);

    return std::make_tuple(*img, euclid, p);
}
