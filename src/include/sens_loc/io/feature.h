#ifndef FEATURE_H_9IGYEWVQ
#define FEATURE_H_9IGYEWVQ

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

namespace sens_loc::io {

inline cv::FileStorage open_feature_file(const std::string& f_path) {
    using cv::FileStorage;
    const FileStorage fs{f_path, FileStorage::READ | FileStorage::FORMAT_YAML};
    return fs;
}

inline std::vector<cv::KeyPoint> load_keypoints(const cv::FileStorage& fs) {
    const cv::FileNode        descriptors_node = fs["keypoints"];
    std::vector<cv::KeyPoint> k;
    cv::read(descriptors_node, k);
    return k;
}

inline cv::Mat load_descriptors(const cv::FileStorage& fs) {
    const cv::FileNode descriptors_node = fs["descriptors"];
    cv::Mat            d;
    cv::read(descriptors_node, d);
    return d;
}

}  // namespace sens_loc::io

#endif /* end of include guard: FEATURE_H_9IGYEWVQ */
