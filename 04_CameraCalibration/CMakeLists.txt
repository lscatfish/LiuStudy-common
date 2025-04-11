#设置当前cmake运行的最低版本
cmake_minimum_required(VERSION 3.2)
 
# 设置项目名称
project(calibration)
 
# 设置C++标准为11
set(CMAKE_CXX_STANDARD 11)
 
# 查找OpenCV库
find_package(OpenCV REQUIRED)
 
#包含OpenCV的头文件
include_directories(${OpenCV_INCLUDE_DIRS})
 
# 添加可执行文件
add_executable(calibration src/Calibration.cpp)
 
# 链接OpenCV库
target_link_libraries(calibration ${OpenCV_LIBS})