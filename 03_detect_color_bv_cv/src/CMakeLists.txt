cmake_minimum_required(VERSION 3.0)
project(ColorDetection)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(color_detection detect_color.cpp)

# 链接OpenCV库到你的项目
target_link_libraries(color_detection ${OpenCV_LIBS})