#ifndef DESCRIPTOR_CHARACTERISTICA_H_MF5XUVPN
#define DESCRIPTOR_CHARACTERISTICA_H_MF5XUVPN

namespace sens_loc::apps {
/// Helper struct that identifies a descriptor in a dataset.
struct descriptor_id {
    int img_idx;    ///< In which file (=image) this descriptor was detected.
    int descr_nbr;  ///< Index of the descriptor local to the file.
};

/// Little helper to encode the minimal distance of a descriptor to the closest
/// descriptor within the same image.
struct descriptor_info_min_dist : descriptor_id {
    float min_distance;  ///< Distance to the next closest descriptor within
                         ///< that file.
};

}  // namespace sens_loc::apps

#endif /* end of include guard: DESCRIPTOR_CHARACTERISTICA_H_MF5XUVPN */
