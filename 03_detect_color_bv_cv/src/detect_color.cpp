#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

void detectBlueYellow(const char *imagePath, const std::string &winname) {
    // 读取图像
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty( )) {
        printf("Error: Image not found.\n");
        return;
    }

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 定义蓝色的HSV范围
    cv::Scalar lower_blue(90, 62, 69);
    cv::Scalar upper_blue(140, 255, 250);
    cv::Mat    mask1;
    cv::inRange(hsv, lower_blue, upper_blue, mask1);
    cv::Mat mask_blue = mask1;

    // 定义黄色的HSV范围
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::Mat    mask_yellow;
    cv::inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

    // 合并蓝色和黄色的掩码
    cv::Mat mask = mask_blue | mask_yellow;

    // 找到轮廓
    std::vector< std::vector< cv::Point > > contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 在原始图像上绘制轮廓
    for (size_t i = 0; i < contours.size( ); i++) {
        if (cv::contourArea(contours[i]) > 100) {    // 过滤掉小的噪声
            cv::Rect rect = cv::boundingRect(contours[i]);
            cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
        }
    }

    // 显示结果
    cv::imshow(winname, image);
    cv::waitKey(0);
    cv::destroyAllWindows( );
}

int main( ) {
    detectBlueYellow("../cube_in.png", "in");
    detectBlueYellow("../cube_out.png", "out");
    return 0;
}