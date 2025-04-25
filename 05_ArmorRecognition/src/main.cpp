#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <stdlib.h>
#include "recognition.h"

//读取图片
//@param 文件路径
//@param 输入图片
//@return 文件不存在false，存在true
bool readImg(const std::string &path, cv::Mat &input) {
    cv::Mat input_ = cv::imread(path);
    if (input_.empty( )) {
        std::cout << std::endl
                  << "路径" << path << "下的文件不存在，推出程序" << std::endl;
        return false;
    }
    input = input_;
    return true;
}

//输入要检测的照片路径
//@param 文件路径
//@return 文件名字正确返回true
bool inputFileName(std::string &path, std::string &color) {
    std::string fileName;

    std::cout << std::endl
              << "请输入文件名字（red/bule）（不用加后缀）：";
    std::cin >> fileName;
    color = fileName;

    if (fileName != "blue" && fileName != "red") {
        std::cout << std::endl
                  << "选择错误，退出程序...";
        return false;
    } else {
        path = "../img/" + fileName + ".png";
        return true;
    }
}

int main( ) {
    std::string path, color;
    if (!inputFileName(path, color)) {
        return -1;
    }

    cv::Mat input;
    if (!readImg(path, input)) {
        return -1;
    }

    cv::Mat     output;
    double      distance;
    recognition i1(input, color, output, distance);

    cv::namedWindow("result");
    cv::imshow("result", output);
    cv::waitKey( );
    cv::destroyAllWindows( );
    i1.~recognition( );
}