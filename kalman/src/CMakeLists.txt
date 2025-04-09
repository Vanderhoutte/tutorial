cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(1+1)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(1+1 1+1.cpp)

# 链接 OpenCV 库
target_link_libraries(1+1 ${OpenCV_LIBS})