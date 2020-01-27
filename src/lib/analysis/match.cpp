#include <gsl/gsl>
#include <sens_loc/analysis/match.h>

namespace sens_loc::analysis {

using namespace cv;
using namespace gsl;
using namespace std;

pair<vector<KeyPoint>, vector<KeyPoint>>
gather_matches(span<const DMatch>   matches,
               span<const KeyPoint> kp0,
               span<const KeyPoint> kp1) noexcept {
    Expects(kp0.size() >= matches.size());
    Expects(kp1.size() >= matches.size());

    vector<KeyPoint> p0(matches.size());
    vector<KeyPoint> p1(matches.size());

    size_t match_idx = 0;
    for (const DMatch& match : matches) {
        p0[match_idx] = kp0[match.queryIdx];
        p1[match_idx] = kp1[match.trainIdx];
        ++match_idx;
    }

    Ensures(p0.size() == p1.size());
    Ensures(p0.size() == matches.size());

    return make_pair(p0, p1);
}
}  // namespace sens_loc::analysis
