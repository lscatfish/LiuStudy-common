#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( ) {
    // 相机内参
    Mat cameraMatrix = (Mat_< double >(3, 3) << 1462.3697, 0, 398.59394,
                        0, 1469.68385, 110.68997,
                        0, 0, 1);

    // 相机畸变参数
    Mat distCoeffs = (Mat_< double >(5, 1) << 0.003518, -0.311778, -0.016581, 0.023682, 0);

    // 物体的三维坐标（单位：厘米）
    vector< Point3f > objectPoints;
    objectPoints.push_back(Point3f(00, 00, 0));
    objectPoints.push_back(Point3f(10, 00, 0));
    objectPoints.push_back(Point3f(10, 10, 0));
    objectPoints.push_back(Point3f(00, 10, 0));

    // 图像上的二维坐标（单位：像素）
    vector< Point2f > imagePoints;
    imagePoints.push_back(Point2f(100, 150));
    imagePoints.push_back(Point2f(300, 150));
    imagePoints.push_back(Point2f(300, 300));
    imagePoints.push_back(Point2f(100, 300));

    // 求解PnP
    Mat rvec, tvec;
    int flags = cv::SOLVEPNP_ITERATIVE;
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, flags);

    // 输出旋转向量和平移向量
    cout << "旋转向量 (rvec): " << rvec << endl;
    cout << "平移向量 (tvec): " << tvec << endl;

    // 计算机器人距离摄像头的距离（单位：米）
    double distance = norm(tvec) / 1000;    // 单位转化：将单位从毫米转换为米
    cout << "机器人距离摄像头的距离: " << distance << " 米" << endl;

    return 0;
}