template <typename Intrinsic>
bool max_curve_converter<Intrinsic>::process_file(
    math::image<double> depth_image, int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    const auto max_curve =
        depth_to_max_curve<double, double>(depth_image, this->intrinsic);
    bool success =
        cv::imwrite(fmt::format(this->_files.output, idx),
                    convert_max_curve<double, ushort>(max_curve).data());

    return success;
}
