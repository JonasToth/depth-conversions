include(benchmark_macros)

create_bm(conversion_bearing conversion/bm_bearing.cpp)
create_bm(conversion_curvature conversion/bm_curvature.cpp)
create_bm(conversion_laser conversion/bm_laser.cpp)

configure_file(conversion/data0-depth.png conversion/data0-depth.png COPYONLY)
