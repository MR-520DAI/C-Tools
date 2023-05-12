cmake_minimum_required(VERSION 3.0)
project(readply)

add_library(mylib STATIC IMPORTED)
set_target_properties(mylib PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/Draw.lib)

include_directories(${PROJECT_SOURCE_DIR}/inc)

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_SOURCE_DIR}/lib/Draw.lib)
