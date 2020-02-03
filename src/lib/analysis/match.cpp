#include <sens_loc/analysis/match.h>
#include <sens_loc/analysis/precision_recall.h>
#include <sens_loc/math/pointcloud.h>

namespace sens_loc::analysis {

using namespace math;
using namespace cv;
using namespace gsl;
using namespace std;

pair<imagepoints_t, imagepoints_t>
gather_correspondences(span<const keypoint_correspondence> matches,
                       const imagepoints_t&                query,
                       const imagepoints_t&                train) noexcept {
    imagepoints_t queried;
    queried.reserve(matches.size());
    imagepoints_t trained;
    trained.reserve(matches.size());

    for (const keypoint_correspondence& match : matches) {
        queried.emplace_back(query.at(match.query_idx));

        if (match.train_idx == -1)
            trained.emplace_back(-1, -1);
        else
            trained.emplace_back(train.at(match.train_idx));
    }
    Ensures(queried.size() == narrow_cast<size_t>(matches.size()));

    return make_pair(queried, trained);
}
}  // namespace sens_loc::analysis
