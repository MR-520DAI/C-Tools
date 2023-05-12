# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(Open3DCMakeFindPackage)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
# set(Open3D_DIR "E:/c_project/tools/Open3D/build/install")

find_package(Open3D REQUIRED)

add_library(Draw Draw.cpp)
target_include_directories(Draw PRIVATE ${Open3D_INCLUDE_DIRS})
target_link_libraries(Draw ${Open3D_LIBRARIES})

add_executable(main main.cpp)
target_link_libraries(main Draw)

