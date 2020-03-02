#include <algorithm>
#include <fstream>
#include <limits>
#include <sens_loc/analysis/recognition_performance.h>
#include <sens_loc/io/feature.h>
#include <sens_loc/io/image.h>
#include <sens_loc/io/pose.h>
#include <sens_loc/math/rounding.h>
#include <unordered_set>

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
    Expects(train_data.size() >= matches.size());
    Expects(query_data.size() < numeric_limits<int>::max());
    Expects(train_data.size() < numeric_limits<int>::max());

    if (query_data.empty() || train_data.empty())
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
    // reuse such a keypoint for a correspondence.
    for (const auto& m : matches) {
        const float px_dist =
            (query_data.at(m.queryIdx) - train_data.at(m.trainIdx)).norm();

        // This match is a true positive
        if (px_dist <= threshold) {
            // Save a true positive.
            true_positives.emplace_back(m.queryIdx, m.trainIdx, m.distance);

            // Disallow the train-pt to be reused by other classifications.
            size_t removed = remaining_train_indices.erase(m.trainIdx);
            // The point could not exist in the remaining_train_indidces.
            // This is due to masking or prior removal.
            Ensures(removed == 1UL || removed == 0UL);
        }
        // This match is a false postive. Allow the 'train-pt' to be reused
        // for false negative calculation.
        else
            false_positives.emplace_back(m.queryIdx, m.trainIdx, m.distance);

        // Remove this index from 'all_query_indices' to know which
        // points still need classification.
        size_t removed = remaining_query_indices.erase(m.queryIdx);
        Ensures(removed == 1UL);
    }
    // The number of total points to query is a constant, so the sum of false
    // positives and true positives plus everything that is unclassified needs
    // to be the number of total points to classify.
    Ensures((true_positives.size() + false_positives.size() +
             remaining_query_indices.size()) == query_data.size());

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
}

void recognition_statistic::account(
    const element_categories& classification) noexcept {
    // Relevant elements
    n_true_pos += classification.true_positives.size();
    t_p_per_image.emplace_back(classification.true_positives.size());
    n_false_neg += classification.false_negatives.size();
    f_n_per_image.emplace_back(classification.false_negatives.size());

    // Irrelevant elements
    n_false_pos += classification.false_positives.size();
    f_p_per_image.emplace_back(classification.false_positives.size());
    n_true_neg += classification.true_negatives.size();
    t_n_per_image.emplace_back(classification.true_negatives.size());

    // Keep track for interesting statistical insights and finally
    // histogramming.
    using gsl::narrow_cast;
    _relevant_elements.stat(
        narrow_cast<int64_t>(classification.true_positives.size()) +
        narrow_cast<int64_t>(classification.false_negatives.size()));
    _true_positives.stat(
        narrow_cast<int64_t>(classification.true_positives.size()));
    _false_positives.stat(
        narrow_cast<int64_t>(classification.false_positives.size()));

    // Keep track of the descriptor distances.
    transform(classification.true_positives.begin(),
              classification.true_positives.end(), back_inserter(tp_distance),
              [](const keypoint_correspondence& c) { return c.distance; });
    transform(classification.false_positives.begin(),
              classification.false_positives.end(), back_inserter(fp_distance),
              [](const keypoint_correspondence& c) { return c.distance; });
}

void recognition_statistic::make_histogram() {
    using namespace std;

    // Every category needs to have the same number of entries in the
    // statistics.
    Expects(size(t_p_per_image) == size(f_p_per_image));
    Expects(size(t_p_per_image) == size(t_n_per_image));
    Expects(size(t_p_per_image) == size(f_n_per_image));

    _true_positives.histo =
        boost::histogram::make_histogram(category_statistic::axis_t(
            0, boost::accumulators::max(_true_positives.stat) + 1,
            "True Positives per Frame"));
    _true_positives.histo.fill(t_p_per_image);

    sort(begin(tp_distance), end(tp_distance));
    _tp_descriptor_distance =
        distance(tp_distance, 50, "True Positive Descriptor Distances");

    _false_positives.histo =
        boost::histogram::make_histogram(category_statistic::axis_t(
            0, boost::accumulators::max(_false_positives.stat) + 1,
            "False Positives per Frame"));
    _false_positives.histo.fill(f_p_per_image);

    sort(begin(fp_distance), end(fp_distance));
    _fp_descriptor_distance =
        distance(fp_distance, 50, "False Positives Descriptor Distances");

    _relevant_elements.histo =
        boost::histogram::make_histogram(category_statistic::axis_t(
            0, boost::accumulators::max(_relevant_elements.stat) + 1,
            "Relevant Elements per Frame"));
    vector<int64_t> sum_elements(size(t_p_per_image));
    transform(begin(t_p_per_image), end(t_p_per_image), begin(f_n_per_image),
              begin(sum_elements), plus<int32_t>{});
    _relevant_elements.histo.fill(sum_elements);
}

void write(cv::FileStorage&             fs,
           const std::string&           name,
           const recognition_statistic& s) {
    using gsl::narrow;
    fs << name << "{";
    fs << "relevant_elements" << narrow<int>(s.relevant_elements());
    fs << "irrelevant_elements" << narrow<int>(s.irrelevant_elements());
    fs << "total_elements" << narrow<int>(s.total_elements());
    fs << "selected_elements" << narrow<int>(s.selected_elements());
    fs << "true_positives" << narrow<int>(s.true_positives());
    fs << "false_positives" << narrow<int>(s.false_positives());
    fs << "true_negatives" << narrow<int>(s.true_negatives());
    fs << "false_negatives" << narrow<int>(s.false_negatives());
    fs << "precision" << math::roundn(s.precision(), 4);
    fs << "recall" << math::roundn(s.recall(), 4);
    fs << "fallout" << math::roundn(s.fallout(), 4);
    fs << "sensitivity" << math::roundn(s.sensitivity(), 4);
    fs << "specificity" << math::roundn(s.specificity(), 4);
    fs << "rand_index" << math::roundn(s.rand_index(), 4);
    fs << "youden_index" << math::roundn(s.youden_index(), 4);
    fs << "}";
}

}  // namespace sens_loc::analysis
