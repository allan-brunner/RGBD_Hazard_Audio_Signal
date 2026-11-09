cmake_minimum_required(VERSION 3.16)
project(RGBD_Hazard_Audio_Signal VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimize specifically for the Qualcomm QRB2210 (Quad-core ARM Cortex-A53) on ARM64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    message(STATUS "Targeting ARM Cortex-A53 (Arduino UNO Q QRB2210)")
    add_compile_options(-mcpu=cortex-a53 -O3 -Wall -Wextra)
else()
    add_compile_options(-O3 -Wall -Wextra)
endif()

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenAL REQUIRED)
find_package(Threads REQUIRED)

add_executable(hazard_detector main.cpp vision.cpp audio.cpp)

target_include_directories(hazard_detector PRIVATE
    ${realsense2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OPENAL_INCLUDE_DIR}
)

target_link_libraries(hazard_detector PRIVATE
    ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
    ${OPENAL_LIBRARY}
    Threads::Threads
)
