cmake_minimum_required(VERSION 3.11)
PROJECT(camera calibration)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #used for the autocomplete YouCompletePlugin for Vundle

ADD_EXECUTABLE(cameracalibration cameracalibration.cpp)
TARGET_LINK_LIBRARIES(camercalibration ${OpenCV_LIBS})
