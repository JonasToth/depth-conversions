template <typename Intrinsic>
bool flexion_converter<Intrinsic>::process_file(math::image<double> depth_image,
                                                int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    const auto flexion =
        depth_to_flexion<double, double>(depth_image, this->intrinsic);
    ;
    bool success = cv::imwrite(fmt::format(this->_files.output, idx),
                               convert_flexion<double, ushort>(flexion).data());

    return success;
}
