cmake_minimum_required(VERSION 2.8)
project(objtrack)

set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
set(NCNN_LINK_LIBS /workspace/ncnn/build/install/lib/libncnn.a)
set(NCNN_INCLUDE_DIRS /workspace/ncnn/build/install/include/ncnn)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${NCNN_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(objtrack STATIC ./src/objdetect.cpp
                            ./src/kalmantrack.cpp)
target_link_libraries(objtrack ${NCNN_LINK_LIBS} ${OpenCV_LIBS})

add_executable(main ./src/main.cpp)
target_link_libraries(main objtrack)