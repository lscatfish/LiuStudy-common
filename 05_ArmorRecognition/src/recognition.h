#ifndef RECOGNITION_H
#define RECOGNITION_H

#include <opencv2/opencv.hpp>
#include <vector>

class recognition {

private:
    cv::Mat     inputImg_;    //原图
    std::string color_;       //检测的颜色

    cv::Mat inputImg_hsv_;    // hsv空间
    cv::Mat color_mask_;      //颜色掩码

    std::vector< std::vector< cv::Point > > contours_;     // 存储所有轮廓
    std::vector< cv::Vec4i >                hierarchy_;    // 存储轮廓的层次结构

    std::vector< cv::Rect > light_strips_;    // 符合条件的灯条轮廓矩形
    std::vector< cv::Rect > best_strips_;     // 最佳的两个灯条轮廓矩形

    double distance_;    //距离相机的距离

    const double mergeThreshold_ = 8;    // 融合阈值，可调整

    // 旋转向量
    cv::Mat rvec_;
    // 平移向量
    cv::Mat tvec_;

    //输出图像
    cv::Mat outputImg_;

    // #################################################################

    void filterAndCvt( );
    void setColorMask( );
    void mergeCloseContours( );
    void filterLightStrips( );
    void selectBestLightStrips( );
    void calculateDistance( );
    void printDistance( );
    void setTangle( );

    // #################################################################

    static cv::Mat
    createColorMask(
        const cv::Mat    &hsv,
        const cv::Scalar &lower,
        const cv::Scalar &upper);
    static double contourDistance(
        const std::vector< cv::Point > &contour1,
        const std::vector< cv::Point > &contour2);

public:
    //世界系
    const std::vector< cv::Point3f > obj_points = {
        cv::Point3f(-40, -20, 0),    // 左灯条一端
        cv::Point3f(-40, 20, 0),     // 左灯条另一端
        cv::Point3f(40, -20, 0),     // 右灯条一端
        cv::Point3f(40, 20, 0)       // 右灯条另一端
    };
    // 相机内参矩阵（单位：mm）
    const cv::Mat camera_matrix = (cv::Mat_< double >(3, 3) << 1462.3697, 0, 398.59394, 0, 1469.68385, 110.68997, 0, 0, 1);
    // 畸变矩阵
    const cv::Mat dist_coeffs = (cv::Mat_< double >(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.00000);

    recognition(
        const cv::Mat     &input,
        const std::string &color,
        cv::Mat           &output,
        double            &distance);
    ~recognition( );
};



#endif