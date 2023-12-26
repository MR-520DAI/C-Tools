cmake_minimum_required(VERSION 3.0)
project(main)

set(CMAKE_CXX_STANDARD 14)  
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(PYTHON_EXECUTABLE D:/software/Miniconda3/envs/opencv/python)  
set(PYTHON_INCLUDE_DIR D:/software/Miniconda3/envs/opencv/include)  
set(PYTHON_LIBRARY D:/software/Miniconda3/envs/opencv/libs/python38.lib)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARY})

add_executable(main main.cpp)  
target_link_libraries(main ${PYTHON_LIBRARIES})
