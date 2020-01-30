#include <cstddef>
#include <gsl/gsl>
#include <iostream>
#include <sens_loc/analysis/match.h>

namespace sens_loc::analysis {

using namespace cv;
using namespace gsl;
using namespace std;

pair<vector<KeyPoint>, vector<KeyPoint>>
gather_matches(const vector<DMatch>&   matches,
               const vector<KeyPoint>& query,
               const vector<KeyPoint>& train) noexcept {
    vector<KeyPoint> queried;
    queried.reserve(matches.size());
    vector<KeyPoint> trained;
    trained.reserve(matches.size());

    for (const DMatch& match : matches) {
        queried.emplace_back(query.at(match.queryIdx));
        trained.emplace_back(train.at(match.trainIdx));
    }

    Ensures(queried.size() == trained.size());
    Ensures(queried.size() == matches.size());

    return make_pair(queried, trained);
}
}  // namespace sens_loc::analysis
