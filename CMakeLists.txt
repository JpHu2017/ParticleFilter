cmake_minimum_required(VERSION 2.8)
project(particlefilter)

#find_package(OpenCV 2.4.13 REQUIRED COMPONENTS core imgproc highgui)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV 2.4.13 REQUIRED)

include_directories(${OpenCV_INCLUDE_PATH})

add_executable(particlefilter main.cpp JpParticleFilter.cpp JpParticleFilter.h JpParticleFilter2.cpp JpParticleFilter2.h)

target_link_libraries(particlefilter ${OpenCV_LIBRARIES})

