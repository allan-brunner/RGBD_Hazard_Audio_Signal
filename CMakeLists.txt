cmake_minimum_required(VERSION 3.10)
project(HEADSoftware)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 REQUIRED)

add_executable(head_soft main.cpp)
target_link_libraries(head_soft PRIVATE realsense2::realsense2)
