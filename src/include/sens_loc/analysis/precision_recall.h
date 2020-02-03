#ifndef PRECISION_RECALL_H_0COUZXGS
#define PRECISION_RECALL_H_0COUZXGS

#include <iterator>
#include <limits>
#include <opencv2/features2d.hpp>
#include <sens_loc/camera_models/pinhole.h>
#include <sens_loc/camera_models/projection.h>
#include <sens_loc/math/image.h>
#include <sens_loc/math/pointcloud.h>
#include <unordered_set>

namespace sens_loc::analysis {

/// Store a keypoint correspondence with indices into the original keypoints.
struct keypoint_correspondence {
    int query_idx;
    int train_idx;

    keypoint_correspondence(int query, int train) noexcept
        : query_idx{query}
        , train_idx{train} {}
};

/// Store the classification of keypoints in either:
/// - true positive  -> the keypoint has been matched correctly
/// - false positive -> the keypoint has been matched, but the a wrong keypoint
/// - false negative -> the keypoint has a corresponding keypoint but did not
/// match
/// - true negative  -> the keypoint does not have a correspondence which is
/// detected correctly
///
/// The measure for "correct" or "incorrect" is the backprojection error in
/// euclidean pixel distance.
/// The correspondences go from "query" to "train". If "query" does not have a
/// corresponding "train", the \c train_idx will be \c -1
/// \note the \c absolute_pose of the provided information is not used, because
/// the relative pose can be refined with an ICP yielding a higher precision.
struct element_categories {
    std::vector<keypoint_correspondence> true_positives;
    std::vector<keypoint_correspondence> false_positives;
    std::vector<keypoint_correspondence> false_negatives;
    std::vector<keypoint_correspondence> true_negatives;

    double precision = 0.;
    double recall    = 0.;

    /// Classify every point in 'query_data' according to the allowed pixel
    /// error \c threshold.
    /// \param query_data set of keypoints that are detected in an image
    /// \param train_data set of keypoints that are used for matching with \c
    /// query_data
    /// \param matches result of externally matching \c query_data
    /// and \c train_data descriptors. Elements in that set are the 'selected
    /// elements' for the precision-recall set.
    /// \param threshold in pixel-distance for two point to be relevant (== to
    /// be considered an actual correspondence of keypoints)
    /// \pre threshold > 0.0F
    /// \pre query_data.size() >= matche.size()
    /// \pre query_data.size() < numeric_limits<int>::max() because the
    /// correspondence is stored as an int. The same constraints i put onto
    /// train_data.
    /// \post correspondences are stored as indices into \c
    /// query_data and \c train_data
    /// \post index {-1} is used to signal that no correspondence exists.
    /// \post (true_positives.size() + false_positives.size() +
    /// false_negatives.size() + true_negatives.size()) == query_data.size()
    element_categories(const math::imagepoints_t&     query_data,
                       const math::imagepoints_t&     train_data,
                       const std::vector<cv::DMatch>& matches,
                       float                          threshold) noexcept;
};

}  // namespace sens_loc::analysis

#endif /* end of include guard: PRECISION_RECALL_H_0COUZXGS */
