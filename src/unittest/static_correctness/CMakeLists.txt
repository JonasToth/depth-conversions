# Code snippets that shall not compile and need to be checked to actually not
# compile.
compile_failure(coordinate coordinate.cpp ASSIGN_DIFFERENT_REAL_TYPE)
compile_failure(coordinate coordinate.cpp ASSIGN_DIFFERENT_SYSTEM)
compile_failure(coordinate coordinate.cpp ACCESS_CONVENTION)
compile_failure(coordinate coordinate.cpp ACCESS_CROSS_PRODUCT_2D)
compile_failure(coordinate coordinate.cpp OPERATOR_SCALING_PIXEL)
compile_failure(coordinate coordinate.cpp OPERATOR_SCALING_IMAGE)
compile_failure(coordinate coordinate.cpp OPERATOR_SCALING_WORLD)
compile_failure(coordinate coordinate.cpp SPHERICAL_SCALING_TO_SPHERICAL)
