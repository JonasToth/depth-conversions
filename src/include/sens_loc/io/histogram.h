#ifndef HISTOGRAM_H_AQORBGAS
#define HISTOGRAM_H_AQORBGAS

#include <boost/histogram.hpp>
#include <sstream>
#include <string>

namespace sens_loc::io {

template <typename Histogram>
std::string to_gnuplot(const Histogram& h) {
    std::ostringstream os;

    if (h.rank() == 2U) {
        // Output comment for each columns meaning
        os << "# Width\tHeight\tCount\n";

        // The second index is running slow, so that needs to be check for
        // line separation, which is expected by gnuplot.
        int previous_index = 0;
        for (auto&& cell : boost::histogram::indexed(h)) {
            // Add a additional new line if there was a jump in the slow running
            // index.
            os << (cell.index(1) > previous_index ? "\n" : "");
            os << cell.index(0) << "\t" << cell.index(1) << "\t" << *cell
               << "\n";
            previous_index = cell.index(1);
        }
    } else if (h.rank() == 1U) {
        // Output comment for each columns meaning
        os << "# Lower Bound\tCount\tWidth\n";

        for (auto&& cell : boost::histogram::indexed(h))
            os << cell.bin(0).lower() << "\t" << *cell << "\t"
               << cell.bin(0).upper() - cell.bin(0).lower() << "\n";
    } else {
        throw std::invalid_argument{
            "Only 1D and 2D histograms are supported for output to gnuplot"};
    }

    return os.str();
}

}  // namespace sens_loc::io

#endif /* end of include guard: HISTOGRAM_H_AQORBGAS */
