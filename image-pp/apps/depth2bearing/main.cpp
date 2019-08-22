#include <iostream>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char **argv)
{
    auto Image = cv::imread("/home/jonas/owncloud/Freiberg/Masterarbeit/full_data_sets/lehrpfad/raw_data/data196.jpg");
    cv::imwrite("/home/jonas/data196.png", Image);
    std::cout << "Whats upp\n";
    return 0;
}
