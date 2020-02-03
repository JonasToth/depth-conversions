#include <fstream>
#include <limits>
#include <sens_loc/analysis/precision_recall.h>
#include <sens_loc/io/feature.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/pose.h>

using namespace std;
using namespace cv;

namespace sens_loc::analysis {

element_categories::element_categories(const math::imagepoints_t& query_data,
                                       const math::imagepoints_t& train_data,
                                       const std::vector<cv::DMatch>& matches,
                                       float threshold) noexcept {
    using namespace std;
    using namespace cv;
    using namespace math;

    Expects(threshold > 0.0F);
    Expects(query_data.size() >= matches.size());
    Expects(query_data.size() < numeric_limits<int>::max());
    Expects(train_data.size() < numeric_limits<int>::max());

    if (query_data.empty())
        return;

    const int query_data_size = gsl::narrow_cast<int>(query_data.size());
    const int train_data_size = gsl::narrow_cast<int>(train_data.size());
    Ensures(query_data_size > 0);
    Ensures(train_data_size > 0);

    // This set contains all (unclassified) possible indices into
    // 'query_data'.
    unordered_set<int> remaining_query_indices;

    // This set contains all positivly used indices into 'train_data'.
    // This accounting ensures, that no keypoints are used
    // twice for classification of a query point.
    // This can happen, if a keypoint is close enough to its matched
    // keypoint but another unmatched keypoint is closer or other similar
    // pathological cases.
    // These cases are excluded with masking already used positive train
    // points.
    unordered_set<int> remaining_train_indices;

    for (int i = 0; i < query_data_size; ++i)
        remaining_query_indices.emplace(i);
    for (int i = 0; i < train_data_size; ++i)
        remaining_train_indices.emplace(i);

    // 1. Matches need to be classified in either true or false positives.
    // True positives remove the corresonding train-point from further
    // matching. This ensures, that the "false-negative" stage does not
    // "reuse" such a keypoint for a correspondence.
    for (const auto& m : matches) {
        const float px_dist =
            (query_data[m.queryIdx] - train_data[m.trainIdx]).norm();

        // This match is a true positive
        if (px_dist <= threshold) {
            // Save a true positive.
            true_positives.emplace_back(m.queryIdx, m.trainIdx);

            // Disallow the train-pt to be reused by other classifications.
            size_t removed = remaining_train_indices.erase(m.trainIdx);
            Ensures(removed == 1UL || removed == 0UL);
        }
        // This match is a false postive. Allow the 'train-pt' to be reused
        // for false negative calculation.
        else
            false_positives.emplace_back(m.queryIdx, m.trainIdx);

        // Remove this index from 'all_query_indices' to know which
        // points still need classification.
        size_t removed = remaining_query_indices.erase(m.queryIdx);
        Ensures(removed == 1UL);
    }
    Ensures(!true_positives.empty() || !false_positives.empty());

    // 2. Search for false negatives in the remain query-points.
    // Note: This has quadratic complexity, because there is a linear search
    // for each remaining train index.
    vector<float> all_distances(train_data.size(),
                                numeric_limits<float>::max());
    for (int i : remaining_query_indices) {
        // There are no possible correspondences anymore.
        // This implies that each remain index in the query-set is a
        // true negative.
        if (remaining_train_indices.empty())
            break;

        // Find the closest point to 'p' in the remaining training set.
        // The distance to every point must be calculated, because there
        // can be multiple close points. The closest is used as
        // correspondence.
        const pixel_coord<float> p = query_data.at(i);
        all_distances.assign(
            std::distance(begin(all_distances), end(all_distances)),
            numeric_limits<float>::max());

        for (int train_idx : remaining_train_indices) {
            if (train_data.at(train_idx).u() == -1 ||
                train_data.at(train_idx).v() == -1)
                continue;
            all_distances.at(train_idx) = (p - train_data.at(train_idx)).norm();
        }
        Ensures(!remaining_train_indices.empty());
        auto min_it = min_element(begin(all_distances), end(all_distances));
        const float min_dist = *min_it;

        // This can happen, if all remaining query-points are invalid
        // == {-1, -1}. In that case the point will be a true negative
        // and will be classified AFTER this loop with the
        // 'remaining points' becoming true negatives.
        if (min_dist == numeric_limits<float>::max())
            continue;

        Ensures(min_dist < numeric_limits<float>::max());

        // The point is a false negative, because there is a close (enough)
        // keypoint in the training set.
        if (min_dist < threshold) {
            const size_t t_idx = std::distance(begin(all_distances), min_it);
            Ensures(t_idx < train_data.size());

            false_negatives.emplace_back(i, t_idx);
            const size_t removed = remaining_train_indices.erase(t_idx);
            Ensures(removed == 1UL || removed == 0UL);
        }
    }
    // At this point 'false_negatives' are filtered out. For every false
    // negative there is an entry in 'false_negatives'. Each 'query_idx'
    // must be removed from 'remaining_query_indices'.
    // The indices, that are left in 'remaining_query_indices' are the
    // true negatives. These are inserted in their category afterwards.
    for (const keypoint_correspondence& corresponce : false_negatives) {
        size_t count = remaining_query_indices.erase(corresponce.query_idx);
        Ensures(count == 1UL);
    }
    for (int idx : remaining_query_indices)
        true_negatives.emplace_back(idx, -1);

    // Every keypoint that has been queried must be in either category.
    Ensures((true_positives.size() + false_positives.size() +
             false_negatives.size() + true_negatives.size()) ==
            query_data.size());

    using gsl::narrow_cast;
    precision =
        narrow_cast<double>(true_positives.size()) /
        narrow_cast<double>(true_positives.size() + false_positives.size());
    recall =
        narrow_cast<double>(true_positives.size()) /
        narrow_cast<double>(true_positives.size() + false_negatives.size());
}


}  // namespace sens_loc::analysis
