cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(EIGEN3_INCLUDE_DIR "E:/Code/VSCode/library")

set(OpenCV_DIR "E:/Code/VSCode/library/opencv_4.8.0/opencv-build/install/x64/mingw/lib")
find_package(OpenCV REQUIRED)
# 确认 OpenCV 是否被正确找到
if(Opencv_FOUND)
     message(STATUS "The Opecv lib is found!") 
 endif()
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

set(CMAKE_CXX_STANDARD 11)

#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

#链接库文件
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})
