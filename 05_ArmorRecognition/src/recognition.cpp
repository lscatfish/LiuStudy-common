#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <stdlib.h>
#include "recognition.h"
#include <cmath>

//######################################################################################

recognition::recognition(const cv::Mat &input, const std::string &color, cv::Mat &output, double &distance) {
    inputImg_ = input.clone( );
    color_    = color;
    filterAndCvt( );
    setColorMask( );

    // 使用RETR_TREE模式检索所有轮廓，包含内部轮廓
    cv::findContours(color_mask_, contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    mergeCloseContours( );
    filterLightStrips( );
    selectBestLightStrips( );
    calculateDistance( );
    printDistance( );
    setTangle( );

    output = outputImg_.clone( );

    std::cout << std::endl
              << tvec_ << std::endl;
}

recognition::~recognition( ) {
}

//######################################################################################



//滤波器 并且转化为hsv
void recognition::filterAndCvt( ) {
    // 存储高斯模糊后的图像
    cv::Mat blurred;
    // 对图像进行高斯模糊降噪
    cv::GaussianBlur(inputImg_, blurred, cv::Size(5, 5), 0);

    cv::Mat medianFiltered;
    // 对高斯模糊后的图像进行中值滤波降噪
    cv::medianBlur(blurred, medianFiltered, 5);

    // 将中值滤波后的图像从BGR颜色空间转换为HSV颜色空间
    cv::cvtColor(medianFiltered, inputImg_hsv_, cv::COLOR_BGR2HSV);
}

//生成颜色掩码
void recognition::setColorMask( ) {
    if (color_ == "red") {
        // 创建低范围红色掩码
        cv::Mat red_mask1 = createColorMask(inputImg_hsv_, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255));
        // 创建高范围红色掩码
        cv::Mat red_mask2 = createColorMask(inputImg_hsv_, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255));
        // 合并两个红色掩码
        color_mask_ = red_mask1 | red_mask2;
    } else if (color_ == "blue") {
        // 创建蓝色掩码
        color_mask_ = createColorMask(inputImg_hsv_, cv::Scalar(82, 24, 184), cv::Scalar(100, 255, 255));
    }
}

// 融合接近的轮廓的函数，输入为所有轮廓和融合阈值，输出为融合后的轮廓
void recognition::mergeCloseContours( ) {
    // 复制所有轮廓到合并后的轮廓向量中
    std::vector< std::vector< cv::Point > > mergedContours = contours_;
    // 标记是否进行了合并操作
    bool merged = true;

    // 只要进行了合并操作，就继续循环
    while (merged) {
        // 初始化为未进行合并操作
        merged = false;
        // 遍历所有轮廓
        for (size_t i = 0; i < mergedContours.size( ); ++i) {
            // 遍历剩余的轮廓
            for (size_t j = i + 1; j < mergedContours.size( ); ++j) {
                // 计算当前两个轮廓之间的最小距离
                double dist = contourDistance(mergedContours[i], mergedContours[j]);
                // 如果最小距离小于融合阈值
                if (dist < mergeThreshold_) {
                    // 合并两个轮廓
                    std::vector< cv::Point > combinedContour = mergedContours[i];
                    combinedContour.insert(combinedContour.end( ), mergedContours[j].begin( ), mergedContours[j].end( ));
                    // 移除原来的第二个轮廓
                    mergedContours.erase(mergedContours.begin( ) + j);
                    // 移除原来的第一个轮廓
                    mergedContours.erase(mergedContours.begin( ) + i);
                    // 添加合并后的轮廓
                    mergedContours.push_back(combinedContour);
                    // 标记进行了合并操作
                    merged = true;
                    // 跳出内层循环
                    break;
                }
            }
            // 如果进行了合并操作，跳出外层循环
            if (merged)
                break;
        }
    }
    // 返回融合后的轮廓向量
    contours_ = mergedContours;
}

// 筛选灯条轮廓的函数，输入为所有轮廓，输出为筛选后的灯条轮廓矩形
void recognition::filterLightStrips( ) {
    // 遍历所有轮廓
    for (const auto &cnt : contours_) {
        // 计算当前轮廓的外接矩形
        cv::Rect rect = boundingRect(cnt);
        // 计算外接矩形的长宽比
        float aspect_ratio = (float)rect.height / rect.width;
        // 判断长宽比是否在合适的范围内（2到15之间）
        if (aspect_ratio > 2 && aspect_ratio < 15) {
            // 若符合条件，将该矩形添加到灯条轮廓矩形向量中
            light_strips_.push_back(rect);
        }
    }
}

// 选择长度最接近且最平行的两个灯条的函数，输入为所有灯条轮廓矩形，输出为最佳的两个灯条轮廓矩形
void recognition::selectBestLightStrips( ) {
    // 如果灯条数量少于2个，直接返回空向量
    if (light_strips_.size( ) < 2) {
        return;
    }

    // 初始化最小得分，用于比较不同灯条对的综合得分，初始化为最大值
    double min_score = DBL_MAX;

    // 双重循环遍历所有灯条对
    for (size_t i = 0; i < light_strips_.size( ); ++i) {
        for (size_t j = i + 1; j < light_strips_.size( ); ++j) {
            // 获取当前灯条对中第一个灯条的高度
            double length1 = light_strips_[i].height;
            // 获取当前灯条对中第二个灯条的高度
            double length2 = light_strips_[j].height;
            // 计算两个灯条高度的差值
            double length_diff = std::abs(length1 - length2);
            // 灯条的长度和
            double length_sum = length1 + length2;

            // 计算第一个灯条的中心点
            cv::Point2f center1(
                light_strips_[i].x + light_strips_[i].width / 2,
                light_strips_[i].y + light_strips_[i].height / 2);
            // 计算第二个灯条的中心点
            cv::Point2f center2(
                light_strips_[j].x + light_strips_[j].width / 2,
                light_strips_[j].y + light_strips_[j].height / 2);
            // 计算两个中心点在x方向的差值
            double dx = center2.x - center1.x;
            // 计算两个中心点在y方向的差值
            double dy = center2.y - center1.y;
            // 计算两个灯条之间的角度
            double angle = std::atan2(dy, dx) * 180 / CV_PI;

            // 计算角度的差值
            double angle_diff = std::abs(angle);

            // 计算灯条间距与灯条长度和的差
            double light_spacing = std::abs((dx - length_sum));

            // 计算综合得分
            double score =
                2 * angle_diff + 8 * length_diff + 12 * abs((light_strips_[i].y - light_strips_[j].y)) + 3 * light_spacing;

            // 如果当前灯条对的综合得分小于最小得分
            if (score < min_score) {
                // 更新最小得分
                min_score = score;
                // 清空之前存储的最佳灯条对
                best_strips_.clear( );
                // 将当前灯条对添加到最佳灯条对向量中
                best_strips_.push_back(light_strips_[i]);
                best_strips_.push_back(light_strips_[j]);
            }
        }
    }
}

void recognition::calculateDistance( ) {
    // 如果灯条数量少于2个，无法计算距离，返回
    if (light_strips_.size( ) < 2) {
        return;
    }
    
    // 构建2D点，获取灯条端点坐标
    std::vector< cv::Point2f > img_points;
    // 左灯条上端点的2D点坐标
    img_points.push_back(cv::Point2f((best_strips_[0].x + best_strips_[0].width) / 2, best_strips_[0].y));
    // 左灯条下端点的2D点坐标
    img_points.push_back(cv::Point2f((best_strips_[0].x + best_strips_[0].width) / 2, best_strips_[0].y + best_strips_[0].height));
    // 右灯条上端点的2D点坐标
    img_points.push_back(cv::Point2f((best_strips_[1].x + best_strips_[1].width) / 2, best_strips_[1].y));
    // 右灯条下端点的2D点坐标
    img_points.push_back(cv::Point2f((best_strips_[1].x + best_strips_[1].width) / 2, best_strips_[1].y + best_strips_[1].height));


    // 调用PnP算法求解相机位姿，返回求解是否成功的标志
    bool success = solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, rvec_, tvec_);

    // 如果求解失败，输出错误信息并返回-1
    if (!success) {
        std::cout << "PnP求解失败" << std::endl;
        return;
    }
    distance_ = cv::norm(tvec_);
}

void recognition::printDistance( ) {
    // 如果距离计算成功
    if (distance_ >= 0) {
        // 输出计算得到的距离
        std::cout << "Distance: " << distance_ << " mm" << std::endl;
    }
}

//绘制矩形
void recognition::setTangle( ) {
    outputImg_ = inputImg_.clone( );
    if (distance_ >= 0) {
        // 在图像上绘制第一个最佳灯条的矩形框
        rectangle(outputImg_, best_strips_[0], cv::Scalar(0, 0, 255), 2);
        // 在图像上绘制第二个最佳灯条的矩形框
        rectangle(outputImg_, best_strips_[1], cv::Scalar(0, 0, 255), 2);
    }
}

// 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
// 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

// 创建颜色掩码的函数，输入为HSV图像、颜色下限和颜色上限，输出为颜色掩码
cv::Mat recognition::createColorMask(const cv::Mat &hsv, const cv::Scalar &lower, const cv::Scalar &upper) {
    cv::Mat mask;                        // 存储颜色掩码
    inRange(hsv, lower, upper, mask);    // 根据颜色范围创建颜色掩码
    return mask;                         // 返回颜色掩码
}

// 计算两个轮廓之间最小距离的函数，输入为两个轮廓，输出为它们之间的最小距离
double recognition::contourDistance(const std::vector< cv::Point > &contour1, const std::vector< cv::Point > &contour2) {
    // 初始化最小距离为最大值
    double minDist = DBL_MAX;

    //算法要更新一下

    // 遍历第一个轮廓的所有点
    for (const auto &p1 : contour1) {
        // 遍历第二个轮廓的所有点
        for (const auto &p2 : contour2) {
            // 计算当前两点之间的距离
            double dist = norm(p1 - p2);
            // 如果当前距离小于最小距离
            if (dist < minDist) {
                // 更新最小距离
                minDist = dist;
            }
        }
    }
    // 返回最小距离
    return minDist;
}
