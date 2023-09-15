cmake_minimum_required(VERSION 2.6)
project(TEST_OPENCV)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIRS})

add_executable(TEST_OPENCV test_opencv.cpp)

target_link_libraries( TEST_OPENCV  ${OpenCV_LIB_DIRS})
