cmake_minimum_required(VERSION 3.0)
project(optiontest)

option(PRINT_TIME "打印时间" ON)
if(PRINT_TIME)
    message("PRINT_TIME:" ${PRINT_TIME})
    add_definitions(-DPRINT_TIME)
endif()

add_executable(main main.cpp)
