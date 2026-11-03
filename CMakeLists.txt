cmake_minimum_required(VERSION 3.10)
project(RealSenseApp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 REQUIRED)

add_executable(realsense_app main.cpp)
target_link_libraries(realsense_app PRIVATE realsense2::realsense2)
