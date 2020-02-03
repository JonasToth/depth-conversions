#ifndef ICP_H_UEHTV2OD
#define ICP_H_UEHTV2OD

#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/math/pointcloud.h>

namespace cv::rgbd {
class Odometry;
};

namespace sens_loc::apps {

template <typename Real = float>
inline cv::Mat cv_camera_matrix(const camera_models::pinhole<Real>& i) {
    cv::Mat K         = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = i.fx();
    K.at<float>(1, 1) = i.fy();
    K.at<float>(0, 2) = i.cx();
    K.at<float>(1, 2) = i.cy();
    return K;
}

/// Refine the pose 'initial_pose' with opencvs icp for pinhole cameras.
/// \returns {refined_pose, icp_successful}. If \c icp_successful is \c false
/// \c refined_pose is the identity matrix.
std::pair<math::pose_t, bool>
refine_pose(cv::rgbd::Odometry&        icp,
            const math::image<ushort>& previous_depth,
            const math::image<ushort>& this_depth,
            double                     unit_factor,
            const math::pose_t&        initial_pose) noexcept;
}  // namespace sens_loc::apps

#endif /* end of include guard: ICP_H_UEHTV2OD */
