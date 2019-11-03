template <typename Intrinsic>
bool flexion_converter<Intrinsic>::process_file(
    const math::image<float>& depth_image, int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    const auto flexion = depth_to_flexion(depth_image, this->intrinsic);
    const bool success = cv::imwrite(fmt::format(this->_files.output, idx),
                                     convert_flexion<ushort>(flexion).data());

    return success;
}
