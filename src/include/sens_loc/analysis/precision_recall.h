#ifndef PRECISION_RECALL_H_0COUZXGS
#define PRECISION_RECALL_H_0COUZXGS

#include <cstdint>
#include <opencv2/features2d.hpp>
#include <sens_loc/math/pointcloud.h>
#include <vector>

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

/// Statistical evaluation for precision-recall of a whole dataset.
class precision_recall_statistic {
  public:
    precision_recall_statistic() = default;

    /// Track true/false negative/positive for each image.
    void account(const element_categories& classification) noexcept;

    [[nodiscard]] std::int64_t relevant_elements() const noexcept {
        return n_true_pos + n_false_neg;
    }
    [[nodiscard]] std::int64_t irrelevant_elements() const noexcept {
        return n_false_pos + n_true_neg;
    }
    [[nodiscard]] std::int64_t total_elements() const noexcept {
        return relevant_elements() + irrelevant_elements();
    }
    [[nodiscard]] std::int64_t selected_elements() const noexcept {
        return n_true_pos + n_false_pos;
    }
    [[nodiscard]] std::int64_t true_positives() const noexcept {
        return n_true_pos;
    }
    [[nodiscard]] std::int64_t false_positives() const noexcept {
        return n_false_pos;
    }
    [[nodiscard]] std::int64_t true_negatives() const noexcept {
        return n_true_neg;
    }
    [[nodiscard]] std::int64_t false_negatives() const noexcept {
        return n_false_neg;
    }

    [[nodiscard]] double precision() const noexcept {
        std::int64_t selected = selected_elements();
        return selected == 0L ? 0.0
                              : gsl::narrow_cast<double>(true_positives()) /
                                    gsl::narrow_cast<double>(selected);
    }
    [[nodiscard]] double recall() const noexcept {
        std::int64_t relevant = relevant_elements();
        return relevant == 0L ? 0.0
                              : gsl::narrow_cast<double>(true_positives()) /
                                    gsl::narrow_cast<double>(relevant);
    }
    [[nodiscard]] double sensitivity() const noexcept { return recall(); }
    [[nodiscard]] double specificity() const noexcept {
        std::int64_t relevant   = relevant_elements();
        std::int64_t irrelevant = irrelevant_elements();
        return irrelevant == 0L ? 0.0
                                : gsl::narrow_cast<double>(relevant) /
                                      gsl::narrow_cast<double>(irrelevant);
    }
    [[nodiscard]] double rand_index() const noexcept {
        std::int64_t total = total_elements();
        return total == 0L ? 0.0
                           : gsl::narrow_cast<double>(true_positives() +
                                                      true_negatives()) /
                                 gsl::narrow_cast<double>(total);
    }
    [[nodiscard]] double youden_index() const noexcept {
        return sensitivity() + specificity() - 1.0;
    }

  private:
    // Keep track on the true/false positives/negatives per image.
    std::vector<std::int64_t> t_p_per_image;
    std::vector<std::int64_t> f_p_per_image;
    std::vector<std::int64_t> t_n_per_image;
    std::vector<std::int64_t> f_n_per_image;

    // Keep track of the global count of elements.
    std::int64_t n_true_pos  = 0L;
    std::int64_t n_false_pos = 0L;
    std::int64_t n_true_neg  = 0L;
    std::int64_t n_false_neg = 0L;
};

}  // namespace sens_loc::analysis

#endif /* end of include guard: PRECISION_RECALL_H_0COUZXGS */
