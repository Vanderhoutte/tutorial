# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(Camera)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(Camera amor_detector.cpp)

# 链接 OpenCV 库
target_link_libraries(Camera ${OpenCV_LIBS})    