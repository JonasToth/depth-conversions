include(benchmarking)

configure_file(conversion/data0-depth.png conversion/data0-depth.png COPYONLY)
configure_file(conversion/laserscan-depth.png conversion/laserscan-depth.png COPYONLY)

create_bm(conversion_bearing conversion/bm_bearing.cpp)
create_bm(conversion_curvature conversion/bm_curvature.cpp)
create_bm(conversion_flexion conversion/bm_flexion.cpp)
create_bm(conversion_laser conversion/bm_laser.cpp)
