#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>

using namespace cv;
using namespace std;

int main( ) {
    // 1. 准备标定棋盘图像
    int   boardWidth  = 8;      // 棋盘格横向内角点数量
    int   boardHeight = 6;      // 棋盘格纵向内角点数量
    float squareSize  = 1.f;    // 棋盘格格子的大小，单位为米,随便设置，不影响相机内参计算
    Size  boardSize(boardWidth, boardHeight);

    vector< vector< Point3f > > objectPoints;
    vector< vector< Point2f > > imagePoints;
    vector< Point2f >           corners;

    // 2. 拍摄棋盘图像
    Mat image, gray;
    namedWindow("image", WINDOW_NORMAL);
    vector< String > fileNames;
    glob("../images/*.jpg", fileNames);

    if (fileNames.size( ) == 0) {
        cout << endl
             << "文件读取失败......" << endl;
        return 0;
    } else {
        cout << endl
             << "读取到了" << fileNames.size( ) << "份文件..." << endl;
    }

    for (size_t i = 0; i < fileNames.size( ); i++) {
        image = imread(fileNames[i], IMREAD_COLOR);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        // 3. 读入图像数据，并提取角点
        bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(image, boardSize, corners, found);
            imshow("image", image);
            waitKey(500);

            vector< Point3f > objectCorners;
            for (int j = 0; j < boardHeight; j++) {
                for (int k = 0; k < boardWidth; k++) {
                    objectCorners.push_back(Point3f(k * squareSize, j * squareSize, 0));
                }
            }
            objectPoints.push_back(objectCorners);
            imagePoints.push_back(corners);
            cout << "文件" << fileNames[i] << "计算成功" << endl;
        } else {
            cout << "文件" << fileNames[i] << "计算失败" << endl;
        }
    }
    cv::destroyAllWindows( );

    // 4. 标定相机
    Mat           cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    calibrateCamera(objectPoints, imagePoints, image.size( ), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << endl
         << endl;
    cout << "Camera matrix:" << endl
         << cameraMatrix << endl;
    cout << "Distortion coefficients:" << endl
         << distCoeffs << endl;

    return 0;
}