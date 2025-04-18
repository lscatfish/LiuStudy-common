#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main( ) {
    // 现实世界点坐标（mm）
    vector< Point3f > objectPoints;
    objectPoints.push_back(Point3f(000, 000, 0));
    objectPoints.push_back(Point3f(100, 000, 0));
    objectPoints.push_back(Point3f(100, 100, 0));
    objectPoints.push_back(Point3f(000, 100, 0));

    // 图像点坐标（像素）
    vector< Point2f > imagePoints;
    imagePoints.push_back(Point2f(100, 150));
    imagePoints.push_back(Point2f(300, 150));
    imagePoints.push_back(Point2f(300, 300));
    imagePoints.push_back(Point2f(100, 300));

    // 摄像头内参矩阵（mm）
    Mat cameraMatrix = (Mat_< double >(3, 3) << 1462.3697, 0, 398.59394,
                        0, 1469.68385, 110.68997,
                        0, 0, 1);

    // 畸变系数矩阵（mm）
    Mat distCoeffs = (Mat_< double >(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0);

    // 旋转向量和平移向量
    Mat rvec, tvec;

    // 调用 solvePnP 计算位姿
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);


    cout << "旋转向量：\n"
         << rvec << endl;

    // 将旋转向量转换为旋转矩阵
    Mat rmat;
    Rodrigues(rvec, rmat);

    cout << "旋转矩阵：\n"
         << rmat << endl;

    // 平移向量（mm，以摄像头为坐标系的相对位移）
    cout << "平移向量（mm）：\n"
         << tvec << endl;

    // 计算机器人到摄像头的距离（m）
    double distance_m = norm(tvec) / 1000.0;    // mm 转换为 m
    cout << "机器人距离摄像头的距离（m）：" << distance_m << endl;

    return 0;
}