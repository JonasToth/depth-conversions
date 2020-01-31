#ifndef MATCH_H_COIEXYNJ
#define MATCH_H_COIEXYNJ

#include <gsl/gsl>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <utility>
#include <vector>

namespace sens_loc::analysis {

/// Create linear vectors that pair the keypoints from \c kp0 to \c kp1
/// according to \c matches.
///
/// \pre \c query is the query set in \c matches.
/// \pre \c train is the training set in \c matches.
/// \param matches Set of matches in \c query and \c train
/// \param query Set of keypoints that were matched as the query set.
/// \param train Set of keypoints that were matched as the training set.
/// \returns \c pair{QueriedKps, TrainedKps} were each vector contains only
/// matched keypoints. All other keypoints are discarded in the result.
std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>
gather_matches(const std::vector<cv::DMatch>&   matches,
               const std::vector<cv::KeyPoint>& query,
               const std::vector<cv::KeyPoint>& train) noexcept;
}  // namespace sens_loc::analysis

#endif /* end of include guard: MATCH_H_COIEXYNJ */
