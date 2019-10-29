template <typename Intrinsic>
bool bearing_converter<Intrinsic>::process_file(math::image<double> depth_image,
                                                int idx) const noexcept {
    Expects(!this->_files.horizontal.empty() ||
            !this->_files.vertical.empty() || !this->_files.diagonal.empty() ||
            !this->_files.antidiagonal.empty());
    using namespace sens_loc::conversion;

    bool final_result = true;
    // NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BEARING_PROCESS(DIRECTION)                                             \
    if (!this->_files.DIRECTION.empty()) {                                     \
        math::image<double> bearing = depth_to_bearing<direction::DIRECTION>(  \
            depth_image, this->intrinsic);                                     \
        bool success =                                                         \
            cv::imwrite(fmt::format(this->_files.DIRECTION, idx),              \
                        convert_bearing<double, ushort>(bearing).data());      \
        final_result &= success;                                               \
    }

    BEARING_PROCESS(horizontal)
    BEARING_PROCESS(vertical)
    BEARING_PROCESS(diagonal)
    BEARING_PROCESS(antidiagonal)

#undef BEARING_PROCESS

    return final_result;
}
