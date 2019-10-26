template <typename Intrinsic>
bool gauss_curv_converter<Intrinsic>::process_file(
    math::image<double> depth_image, int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    const auto gauss = depth_to_gaussian_curvature<double, double>(
        depth_image, this->intrinsic);
    const auto converted =
        conversion::curvature_to_image<double, ushort, double>(
            gauss, depth_image, lower_bound, upper_bound);
    bool success =
        cv::imwrite(fmt::format(this->_files.output, idx), converted.data());

    return success;
}

template <typename Intrinsic>
bool mean_curv_converter<Intrinsic>::process_file(
    math::image<double> depth_image, int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    const auto mean =
        depth_to_mean_curvature<double, double>(depth_image, this->intrinsic);
    const auto converted =
        conversion::curvature_to_image<double, ushort, double>(
            mean, depth_image, lower_bound, upper_bound);
    bool success =
        cv::imwrite(fmt::format(this->_files.output, idx), converted.data());

    return success;
}
