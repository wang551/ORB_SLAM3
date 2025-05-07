cmake_minimum_required(VERSION 3.0)

# 设置项目名称
project(ORB_SLAM3_Read)

# 查找PCL库
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization filters)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 查找Boost库
find_package(Boost REQUIRED COMPONENTS serialization)

# 查找OpenGL库
find_package(OpenGL REQUIRED)

# 查找Pangolin库
find_package(Pangolin REQUIRED)

# 指定C++11编译
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL相关定义和包含目录
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# ORB_SLAM3相关包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/../../
    ${PROJECT_SOURCE_DIR}/../../include
    ${PROJECT_SOURCE_DIR}/../../include/CameraModels
    ${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
    ${PROJECT_SOURCE_DIR}/../../Thirdparty
    ${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2
    ${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(Read Read.cc)

# 链接库
target_link_libraries(Read 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS}
    ${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
    ${EIGEN3_LIBS}
    ${Boost_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    pthread
    -lcrypto
    -lGL -lGLU -lGLEW
)

# 设置输出目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../)