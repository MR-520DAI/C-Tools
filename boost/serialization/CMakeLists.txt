cmake_minimum_required(VERSION 3.0)
project(boostvector)

set(CMAKE_CXX_STANDARD 11)
find_package(Boost REQUIRED COMPONENTS serialization)

add_executable(main main.cpp)
target_link_libraries(main ${Boost_LIBRARIES})
