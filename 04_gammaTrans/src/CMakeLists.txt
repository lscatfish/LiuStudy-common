cmake_minimum_required(VERSION 3.10.0)

# 添加头文件目录
include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(OpenCV REQUIRED)

add_executable(gamma main.cpp)

# 链接OpenCV库到你的项目
target_link_libraries(gamma ${OpenCV_LIBS})
# target_link_librarie
