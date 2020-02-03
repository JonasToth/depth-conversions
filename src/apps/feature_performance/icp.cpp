#include "icp.h"

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/rgbd/depth.hpp>

namespace sens_loc::apps {
std::pair<math::pose_t, bool>
refine_pose(cv::rgbd::Odometry&        icp,
            const math::image<ushort>& previous_depth,
            const math::image<ushort>& this_depth,
            double                     unit_factor,
            const math::pose_t&        initial_pose) noexcept {
    using namespace cv;

    Mat prev_mask;
    previous_depth.data().convertTo(prev_mask, CV_8UC1);

    Mat this_mask;
    this_depth.data().convertTo(this_mask, CV_8UC1);

    Mat cvt_prev_depth;
    previous_depth.data().convertTo(cvt_prev_depth, CV_32F, unit_factor);

    Mat cvt_this_depth;
    this_depth.data().convertTo(cvt_this_depth, CV_32F, unit_factor);

    Mat initial = Mat::eye(4, 4, CV_64FC1);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            initial.at<double>(i, j) = initial_pose(i, j);

    Mat        Rt;
    const bool icp_success = icp.compute(/*srcImage=*/Mat(),
                                         /*srcDepth=*/cvt_prev_depth,
                                         /*srcMask=*/prev_mask,
                                         /*dstImage=*/Mat(),
                                         /*dstDepth=*/cvt_this_depth,
                                         /*dstMask=*/this_mask,
                                         /*Rt=*/Rt,
                                         /*initRt=*/initial);

    math::pose_t result_pose = math::pose_t::Identity(4, 4);
    if (icp_success) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result_pose(i, j) =
                    gsl::narrow_cast<float>(Rt.at<double>(i, j));
    }

    return {result_pose, icp_success};
}
}  // namespace sens_loc::apps
