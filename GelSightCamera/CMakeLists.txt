cmake_minimum_required(VERSION 2.8)
project( GelSightCamera )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_PREFIX_PATH "/usr/local/share/OpenCV")

find_package( OpenCV 3.2 REQUIRED)
include_directories(include)

message(STATUS "OpenCV libs are found: ${OpenCV_LIBS}")

add_executable( testCalibrateGelSightHeightMap 
include/CGelSightGrabImg.h 
include/CGelSightHeightMapCalibration.h 
src/CGelSightGrabImg.cpp 
src/CGelSightHeightMapCalibration.cpp
src/testCalibrateGelSightHeightMap.cpp)
target_link_libraries( testCalibrateGelSightHeightMap ${OpenCV_LIBS})


add_executable( testGelSightCamera 
include/CGelSightGrabImg.h 
include/CGelSightMarkerMotion.h
include/CGelSightHeightMap.h
include/COpenCVPoissonSolver.h 
src/COpenCVPoissonSolver.cpp
src/CGelSightGrabImg.cpp 
src/CGelSightMarkerMotion.cpp 
src/CGelSightHeightMap.cpp
src/testGelSightCamera.cpp)
target_link_libraries( testGelSightCamera ${OpenCV_LIBS})

