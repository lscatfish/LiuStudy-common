#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>

enum ChooseDetectColor {
    dcolor_ALL = 0,
    dcolor_YELLOW,
    dcolor_BLUE,
};

ChooseDetectColor chooseDetectColor( ) {
    int inputc = -1;
    while (!(inputc >= 0 && inputc <= 2)) {
        std::cout << std::endl
                  << "请选择要识别的颜色：0.黄色&蓝色 1.黄色 2.蓝色" << std::endl;
        std::cin >> inputc;
        switch (inputc) {
            case 0:
                return dcolor_ALL;
                break;
            case 1:
                return dcolor_YELLOW;
                break;
            case 2:
                return dcolor_BLUE;
                break;
            default:
                std::cout << std::endl
                          << "你的输入存在错误，请重新输入..." << std::endl;
                break;
        }
    }
    return 0;
}

void detectBlueYellow(
    const char        *imagePath,
    const std::string &winname,
    ChooseDetectColor  whichcolor) {
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

    // 选择检测的颜色
    cv::Mat mask;
    switch (whichcolor) {
        case dcolor_ALL:
            mask = mask_blue | mask_yellow;
            break;
        case dcolor_BLUE:
            mask = mask_blue;
            break;
        case dcolor_YELLOW:
            mask = mask_yellow;
            break;
        default:
            return;
            break;
    }

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
    ChooseDetectColor wc = chooseDetectColor( );
    detectBlueYellow("../cube_in.png", "in", wc);
    detectBlueYellow("../cube_out.png", "out", wc);
    return 0;
}
