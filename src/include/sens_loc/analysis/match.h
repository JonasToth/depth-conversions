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
/// \pre \c kp0 is the query set in \c matches.
/// \pre \c kp1 is the training set in \c matches.
std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>
gather_matches(gsl::span<const cv::DMatch>   matches,
               gsl::span<const cv::KeyPoint> kp0,
               gsl::span<const cv::KeyPoint> kp1) noexcept;
}  // namespace sens_loc::analysis

#endif /* end of include guard: MATCH_H_COIEXYNJ */