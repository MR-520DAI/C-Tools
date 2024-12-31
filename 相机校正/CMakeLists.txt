cmake_minimum_required(VERSION 2.8)

project(image)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Calib 单目标定.cpp)
target_link_libraries(Calib ${OpenCV_LIBS})
