cmake_minimum_required(VERSION 3.0)
project(boostvector)

set(CMAKE_BUILD_TYPE "Release")

# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

set(BOOST_ROOT E:/c_project/tools/boost_1_82_0/build_release)
set(Boost_LIBRARY_DIRS E:/c_project/tools/boost_1_82_0/build_release/lib)

find_package(Boost REQUIRED COMPONENTS container)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${Boost_LIBRARIES})
