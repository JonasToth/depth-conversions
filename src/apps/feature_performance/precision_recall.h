#ifndef PRECISION_RECALL_H_G2FJDYVV
#define PRECISION_RECALL_H_G2FJDYVV

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/base.hpp>
#include <optional>
#include <string_view>

namespace sens_loc::apps {

//  1. Load the pose of frame one and two
//  2. Calculate the relative pose of these two
//  3. Load the original depth image of frame one and two and camera intrinsic
//  4. Project all Points into 3D-Space
//  5. Transform the points of image 2 into the coordinate frame of image 1
//     with the transformation matrix of '2.'.
//  6. Load the Keypoints and Descriptors for image one and two.
//  7. Match them with cross-checking.
//  8. Create the list of Keypoints that matched
//  9. Get the corresponding points from image one and two for these keypoints.
// 10. Calculate the distance between each point-pair
// 11. Insert the distances into a vector.
// 12. Statistical analysis of these distances.

int analyze_precision_recall(std::string_view feature_file_pattern,
                             int              start_idx,
                             int              end_idx,
                             std::string_view depth_image_pattern,
                             std::string_view pose_file_pattern,
                             std::string_view intrinsic_file,
                             cv::NormTypes    matching_norm);

}  // namespace sens_loc::apps

#endif /* end of include guard: PRECISION_RECALL_H_G2FJDYVV */
