cmake_minimum_required(VERSION 3.10)
project(PnPSolver)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(include)

# 添加可执行文件
add_executable(pnp_solver src/main.cpp)

# 链接OpenCV库
target_link_libraries(pnp_solver ${OpenCV_LIBS})