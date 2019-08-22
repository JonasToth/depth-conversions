include(unittest_macros)
create_test(util)
test_add_file(util util/test_console.cpp)

create_test(io)
test_add_file(io io/test_image.cpp)

configure_file(io/example-image.jpg io/example-image.jpg COPYONLY)
configure_file(io/example-image.png io/example-image.png COPYONLY)
configure_file(io/not_an_image.txt io/not_an_image.txt COPYONLY)
