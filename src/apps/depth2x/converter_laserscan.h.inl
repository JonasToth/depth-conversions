template <typename Intrinsic>
bool range_converter<Intrinsic>::process_file(
    const math::image<double>& depth_image, int idx) const noexcept {
    Expects(!this->_files.output.empty());
    using namespace conversion;

    /// The input 'depth_image' is already in range-form as its beeing
    /// preprocessed.
    cv::Mat depth_16bit(depth_image.h(), depth_image.w(), CV_16U);
    depth_image.data().convertTo(depth_16bit, CV_16U);
    bool success =
        cv::imwrite(fmt::format(this->_files.output, idx), depth_16bit);

    return success;
}
