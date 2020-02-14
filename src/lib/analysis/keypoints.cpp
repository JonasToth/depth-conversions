#include <boost/histogram.hpp>
#include <iostream>
#include <sens_loc/analysis/keypoints.h>
#include <sens_loc/math/rounding.h>
#include <sens_loc/util/console.h>
#include <stdexcept>

namespace sens_loc::analysis {

void keypoints::analyze(gsl::span<const cv::KeyPoint> points,
                        const bool                    distribution,
                        const bool                    size,
                        const bool                    response) noexcept {
    _size_histo.reset();
    _size.reset();
    _response_histo.reset();
    _response_histo.reset();
    _distribution.reset();

    if (points.empty())
        return;

    try {
        std::vector<float> responses;
        std::vector<float> sizes;
        std::for_each(points.begin(), points.end(),
                      [&](const cv::KeyPoint& kp) {
                          if (size)
                              sizes.emplace_back(kp.size);
                          if (response)
                              responses.emplace_back(kp.response);
                      });
        if (size) {
            std::sort(std::begin(sizes), std::end(sizes));
            _size = statistic::make(sizes);
        }
        if (response) {
            std::sort(std::begin(responses), std::end(responses));
            _response = statistic::make(responses);
        }
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create statistics for size and response.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }

    using namespace boost::histogram;

    // Shortcut if no histogram will be created.
    if (!distribution && !response && !size)
        return;

    if (distribution)
        try {
            _distribution = make_histogram(
                axis_t{_dist_width_bins, 0.0F, 1.0F, _dist_w_title},
                axis_t{_dist_height_bins, 0.0F, 1.0F, _dist_h_title});
        } catch (const std::invalid_argument& e) {
            std::cerr << sens_loc::util::err{}
                      << "Bad distribution histogram configuration!\n"
                      << "Message: " << e.what() << "\n";
            return;
        }

    const float delta = 5.0F * std::numeric_limits<float>::epsilon();
    if (size && _size_histo_enabled) {
        try {
            const float s_min = _size.min - delta;
            const float s_max = _size.max + delta;
            _size_histo =
                make_histogram(axis_t{_size_bins, s_min, s_max, _size_title});
        } catch (const std::invalid_argument& e) {
            std::cerr
                << sens_loc::util::err{}
                << "Could not create distribution histogram configuration!\n"
                << "Message: " << e.what() << "\n";
            return;
        }
    }

    if (response && _response_histo_enabled) {
        try {
            const float r_min = _response.min - delta;
            const float r_max = _response.max + delta;
            _response_histo   = make_histogram(
                axis_t{_response_bins, r_min, r_max, _response_title});
        } catch (const std::invalid_argument& e) {
            std::cerr << sens_loc::util::err{}
                      << "Could not create response histogram configuration!\n"
                      << "Message: " << e.what() << "\n";
            return;
        }
    }

    try {
        auto iw = static_cast<float>(_img_width);
        auto ih = static_cast<float>(_img_height);
        std::for_each(points.begin(), points.end(),
                      [&](const cv::KeyPoint& kp) {
                          if (distribution)
                              _distribution(kp.pt.x / iw, kp.pt.y / ih);
                          if (response && _response_histo_enabled)
                              _response_histo(kp.response);
                          if (size && _size_histo_enabled)
                              _size_histo(kp.size);
                      });
    } catch (const std::exception& e) {
        std::cerr << sens_loc::util::err{}
                  << "Could not create histograms for size and/or response "
                     "and/or keypoint-distribution.\n"
                  << "Message: " << e.what() << "\n";
        return;
    }
}

void write(cv::FileStorage& fs, const std::string& name, const keypoints& kp) {
    fs << name << "{";
    write(fs, "response", kp.response());
    write(fs, "size", kp.size());
    fs << "}";
}

}  // namespace sens_loc::analysis
