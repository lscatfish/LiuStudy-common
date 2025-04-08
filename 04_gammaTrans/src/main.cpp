#include "opencv2/opencv.hpp"
#include <string>
#include <stdlib.h>
#include <iostream>

//@param 原始图像
//@param gamma变换后的图像
//@param gamma参数
void gammaTransform(const cv::Mat &inputImg, cv::Mat &gammaImg, float gamma) {
    cv::Mat gray, imgfloat64;
    cv::cvtColor(inputImg, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(imgfloat64, CV_64F);
    cv::normalize(imgfloat64, imgfloat64, 0, 1, cv::NORM_MINMAX);
    cv::pow(imgfloat64, gamma, gammaImg);
    cv::normalize(imgfloat64, imgfloat64, 0, 255, cv::NORM_MINMAX);
    imgfloat64.convertTo(imgfloat64, CV_8U);
    return;
}




int main( ) {
    cv::Mat in      = cv::imread("../in1.jpeg");
    cv::Mat in_gray = cv::imread("../in1.jpeg", cv::IMREAD_GRAYSCALE);
    if (in.empty( )) {
        std::cout << "无法读取图像....." << std::endl;
        return 0;
    }
    cv::Mat gammaImg;
    gammaTransform(in, gammaImg, 0.6);
    cv::namedWindow("原灰度图", cv::WINDOW_NORMAL);
    cv::namedWindow("gamma", cv::WINDOW_NORMAL);
    cv::imshow("原灰度图", in_gray);
    cv::imshow("gamma", gammaImg);

    cv::waitKey(0);
    cv::destroyAllWindows( );
}