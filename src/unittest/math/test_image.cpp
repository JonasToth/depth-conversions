#include <doctest/doctest.h>
#include <sens_loc/math/image.h>

using namespace sens_loc::math;

TEST_CASE("image access") {
    SUBCASE("default constructor") { image<unsigned char> def; }

    cv::Mat m(20, 20, CV_16U);
    m = 0;
    // Copy construction with cv::Mat
    image<ushort> i(m);

    SUBCASE("read access") { REQUIRE(i.at(pixel_coord<int>(10, 10)) == 0); }
    SUBCASE("write access") {
        i.at(pixel_coord<int>(10, 10)) = 10;
        REQUIRE(i.at(pixel_coord<int>(10, 10)) == 10);
    }

    i.at(pixel_coord<int>(11, 11)) = 42;
    SUBCASE("copy construction with image") {
        image<ushort> f(i);
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 42);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }


    SUBCASE("move construction with image") {
        image<ushort> my_i(i);
        image<ushort> f(std::move(my_i));
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 42);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }
    SUBCASE("move construction with matrix") {
        cv::Mat m_assign(20, 20, CV_16U);
        m_assign = 0;
        m_assign.at<ushort>(11, 11) = 69;
        image<ushort> f(std::move(m_assign));
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 69);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }


    SUBCASE("copy assignment with image") {
        image<ushort> f;
        f = i;
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 42);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }
    SUBCASE("copy assignment with matrix") {
        image<ushort> f;
        cv::Mat m_assign(20, 20, CV_16U);
        m_assign = 0;
        m_assign.at<ushort>(11, 11) = 69;
        f = m_assign;
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 69);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }


    SUBCASE("move assignment with image") {
        image<ushort> my_i(i);
        image<ushort> f;
        f = std::move(my_i);
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 42);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }
    SUBCASE("move assignment with matrix") {
        image<ushort> f;
        cv::Mat m_assign(20, 20, CV_16U);
        m_assign = 0;
        m_assign.at<ushort>(11, 11) = 69;
        f = std::move(m_assign);
        REQUIRE(f.at(pixel_coord<int>(11, 11)) == 69);
        REQUIRE(f.at(pixel_coord<int>(0, 0)) == 0);
    }

    SUBCASE("underlying data") {
        REQUIRE(i.data().at<ushort>(10, 10) == 0);
    }
}
