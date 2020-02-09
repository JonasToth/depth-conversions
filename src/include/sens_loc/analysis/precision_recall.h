#ifndef PRECISION_RECALL_H_0COUZXGS
#define PRECISION_RECALL_H_0COUZXGS

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/histogram.hpp>
#include <cstdint>
#include <opencv2/features2d.hpp>
#include <sens_loc/math/pointcloud.h>
#include <string>
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

/// Keep track of statistical properties and histogram the appearance of
/// keypoint classifications per image.
/// \sa element_categories
/// \sa precision_recall_statistic
struct category_statistic {
    using accumulator_t = boost::accumulators::accumulator_set<
        std::int64_t,
        boost::accumulators::stats<boost::accumulators::tag::count,
                                   boost::accumulators::tag::min,
                                   boost::accumulators::tag::max,
                                   boost::accumulators::tag::median,
                                   boost::accumulators::tag::mean,
                                   boost::accumulators::tag::variance(
                                       boost::accumulators::lazy)>>;
    using axis_t = boost::histogram::axis::regular<
        // 'float' values are tracked with the histogram.
        float,
        // do no transformation before insertion
        boost::histogram::axis::transform::id,
        // a string is the metadata for the axis (=title)
        std::string,
        // no overflow/underflow by construction.
        boost::histogram::axis::option::none_t>;
    using histo_t = decltype(boost::histogram::make_histogram(axis_t{}));

    accumulator_t stat;
    histo_t       histo;
};

/// Statistical evaluation for precision-recall of a whole dataset.
///
/// This class keeps track of all relevant information to evaluate the
/// performance of the classifier "keypoints are matching".
///
/// \note See https://en.wikipedia.org/wiki/Evaluation_of_binary_classifiers for
/// general information about the statistical relationships.
///
class precision_recall_statistic {
  public:
    precision_recall_statistic() = default;

    /// Track true/false negative/positive for each image.
    void account(const element_categories& classification) noexcept;

    /// Number \f$P\f$ of all keypoints that have a corresponding keypoint in
    /// another frame.
    [[nodiscard]] std::int64_t relevant_elements() const noexcept {
        return n_true_pos + n_false_neg;
    }
    /// Number \f$N\f$ of all keypoints that do NOT have a corresponding
    /// keypoint in another frame.
    [[nodiscard]] std::int64_t irrelevant_elements() const noexcept {
        return n_false_pos + n_true_neg;
    }
    /// Number \f$TP\f$ of keypoints that were correctly identified to have a
    /// correspondence.
    [[nodiscard]] std::int64_t true_positives() const noexcept {
        return n_true_pos;
    }
    /// Number \f$FP\f$ of keypoints that were incorrectly identified to have a
    /// correspondence.
    [[nodiscard]] std::int64_t false_positives() const noexcept {
        return n_false_pos;
    }
    /// Number \f$TN\f$ of keypoints that were correctly revoked to have a
    /// correspondence.
    [[nodiscard]] std::int64_t true_negatives() const noexcept {
        return n_true_neg;
    }
    /// Number \f$FN\f$ of keypoints that do have a correspondence, but did not
    /// get identified to have one.
    [[nodiscard]] std::int64_t false_negatives() const noexcept {
        return n_false_neg;
    }
    /// Number of all keypoints analyzed.
    [[nodiscard]] std::int64_t total_elements() const noexcept {
        return relevant_elements() + irrelevant_elements();
    }
    /// Number \f$TP + FP\f$ of keypoints that were matched. This means the
    /// classifier considers them to have a correspondence.
    [[nodiscard]] std::int64_t selected_elements() const noexcept {
        return n_true_pos + n_false_pos;
    }

    /// The ratio of true positives to total selected elements. The ratio
    /// indicates how many selected elements are relevant.
    //
    /// \f$\frac{TP}{TP + FP}\f$
    /// The higher this values becomes, the lower the false positive ratio is.
    /// \post \f$0.0 <= precision <= 1.0\f$
    [[nodiscard]] double precision() const noexcept {
        std::int64_t selected = selected_elements();
        return selected == 0L ? 0.0
                              : gsl::narrow_cast<double>(true_positives()) /
                                    gsl::narrow_cast<double>(selected);
    }
    /// The ratio of corresponding keypoints to all selected keypoints.
    ///
    /// \f$\frac{TP}{TP + FN}\f$
    /// A high number means, that corresponding keypoints are detected better.
    /// \note this is also called \c sensitivity
    /// \sa sensitivity
    /// \post \f$0.0 <= recall <= 1.0\f$
    [[nodiscard]] double recall() const noexcept {
        std::int64_t relevant = relevant_elements();
        return relevant == 0L ? 0.0
                              : gsl::narrow_cast<double>(true_positives()) /
                                    gsl::narrow_cast<double>(relevant);
    }
    /// The ratio of keypoints that are incorrectly classified of having a
    /// correspondince to all not-corresponding keypoints. This is the false
    /// alarm rate.
    ///
    /// \f$\frac{FP}{TN + FP}\f$
    /// \post \f$0.0 <= fallout <= 1.0\f$
    [[nodiscard]] double fallout() const noexcept {
        std::int64_t irrelevant = irrelevant_elements();
        return irrelevant == 0L ? 0.0
                                : gsl::narrow_cast<double>(false_positives()) /
                                      gsl::narrow_cast<double>(irrelevant);
    }
    /// The same as recall.
    /// \sa recall
    [[nodiscard]] double sensitivity() const noexcept { return recall(); }
    /// The ratio of keypoints that are correctly identified of not having a
    /// correspondence. This is the correct rejection ratio.
    ///
    /// \f$\frac{TN}{TN + FP}\f$
    /// \post \f$0.0 <= specificity <= 1.0\f$
    [[nodiscard]] double specificity() const noexcept {
        std::int64_t irrelevant = irrelevant_elements();
        return irrelevant == 0L ? 0.0
                                : gsl::narrow_cast<double>(true_negatives()) /
                                      gsl::narrow_cast<double>(irrelevant);
    }
    /// The ratio of correct classifications of keypoints into having a
    /// correspondence or not, also called Accuracy
    ///
    /// \f$\frac{TP + TN}{TP + FP + TN + FN}\f$
    /// \post \f$0.0 <= RandIndex <= 1.0\f$
    [[nodiscard]] double rand_index() const noexcept {
        std::int64_t total = total_elements();
        return total == 0L ? 0.0
                           : gsl::narrow_cast<double>(true_positives() +
                                                      true_negatives()) /
                                 gsl::narrow_cast<double>(total);
    }
    /// Probability of an informed decision of the classifier.
    ///
    /// \f$J = sensitivity + specificity - 1\f$
    /// \post \f$-1. <= youden <= 1.\f$
    /// \note Negative values indicate bad labeling, for "normal" conditions
    /// the value will be between \c 0 and \c 1.
    [[nodiscard]] double youden_index() const noexcept {
        return sensitivity() + specificity() - 1.0;
    }

    /// Create the histograms for 'relevant elements', 'true positives' and
    /// 'false positives' to better understand the distribution of these
    /// elements.
    void make_histogram();

    // Make histograms to see the distribution of each element category per
    // image. This allows a judgement of e.g. "how many true positives are at
    // least in an image". This helps ruling out different kinds of algorithms.
    category_statistic _relevant_elements;
    category_statistic _true_positives;
    category_statistic _false_positives;

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
