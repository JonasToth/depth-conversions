#ifndef STATISTIC_VISITOR_H_K2SYXNGM
#define STATISTIC_VISITOR_H_K2SYXNGM

#include <cstdint>
#include <fmt/core.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>

namespace sens_loc::apps {

/// Helper enum that declares which data elements must be loaded for the
/// analysis.
/// The elements can be or'ed (operator|) together to load multiple elements.
/// \sa statistic_visitor
enum class required_data : uint8_t {
    none        = 0,         ///< Do not load any data.
    keypoints   = 1U << 0U,  ///< Load the keypoints.
    descriptors = 1U << 1U,  ///< Load the descriptors.
};
constexpr required_data operator|(required_data element1,
                                  required_data element2) noexcept {
    using T = std::underlying_type_t<required_data>;
    return static_cast<required_data>(static_cast<T>(element1) |
                                      static_cast<T>(element2));
}
constexpr required_data operator&(required_data element1,
                                  required_data element2) noexcept {
    using T = std::underlying_type_t<required_data>;
    return static_cast<required_data>(static_cast<T>(element1) &
                                      static_cast<T>(element2));
}

/// Load the calculated data like keypoints and descriptors in (optionally)
/// and execute the statistical analysis by \c Analysor on it.
/// \tparam Analysor Functor that gets optional<> arguments for each possible
/// datapoint for its overloaded operator(). The gathered data must be stored
/// within that functor.
/// \tparam data_elements bitflags to signal what to load. Saves significant IO.
/// This configuration will be different for each \c Analysor.
/// \warn \c Analysor must be thread safe!
template <typename Analysor, required_data data_elements>
struct statistic_visitor : Analysor {
    static_assert(data_elements != required_data::none,
                  "At least some data must be loaded");

    std::string_view input_pattern;

    template <typename... Args>
    statistic_visitor(std::string_view input_pattern, Args&&... args)
        : Analysor(std::forward<Args>(args)...)
        , input_pattern{input_pattern} {}

    void operator()(int i) noexcept {
        const std::string input_file = fmt::format(input_pattern, i);

        using cv::FileNode;
        using cv::FileStorage;
        using cv::read;
        const FileStorage fs{input_file,
                             FileStorage::READ | FileStorage::FORMAT_YAML};

        std::optional<std::vector<cv::KeyPoint>> keypoints   = std::nullopt;
        std::optional<cv::Mat>                   descriptors = std::nullopt;

        if constexpr ((data_elements & required_data::keypoints) !=
                      required_data::none) {
            const FileNode descriptors_node = fs["keypoints"];

            std::vector<cv::KeyPoint> k;
            read(descriptors_node, k);
            keypoints = k;
        }

        if constexpr ((data_elements & required_data::descriptors) !=
                      required_data::none) {
            const FileNode descriptors_node = fs["descriptors"];

            cv::Mat d;
            read(descriptors_node, d);
            descriptors = d;
        }

        Analysor::operator()(i, std::move(keypoints), std::move(descriptors));
    }
};
}  // namespace sens_loc::apps

#endif /* end of include guard: STATISTIC_VISITOR_H_K2SYXNGM */
