cmake_minimum_required(VERSION 2.6)

project(camera-self-calibration)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11")

set(CMAKE_C_FLAGS_RELEASE "-W -L -O3 ")
set(CMAKE_CXX_FLAGS_RELEASE "-W -L -O3 -std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/build/CMakeModules/")

set(GLUT_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
set(OpenCV_DIR /home/lycheng/Dev/opencv-3.4.0/build/)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(GFLAGS REQUIRED) 
#find_package(GLUT 3.0 REQUIRED)
#find_package(GLEW 2.1 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include 
                    /usr/include 
                    /usr/local/include 
                    ${OpenCV_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS}
                    ${GLOG_INCLUDE_DIRS}
                    ${GFLAGS_INCLUDE_DIRS}
                    )

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    lib path: ${OpenCV_LIBRARY_DIRS}")

message("       ")
message( [Main] " GFLAGS_FOUND = ${GFLAGS_FOUND}")
message( [Main] " GFLAGS_INCLUDE_DIRS = ${GFLAGS_INCLUDE_DIRS}")
message( [Main] " GFLAGS_LIBRARIES = ${GFLAGS_LIBRARIES}")

message("       ")
message( [Main] " GLOG_FOUND = ${GLOG_FOUND}")
message( [Main] " GLOG_INCLUDE_DIRS = ${GLOG_INCLUDE_DIRS}")
message( [Main] " GLOG_LIBRARIES = ${GLOG_LIBRARIES}")

link_directories(/usr/lib /usr/local/lib 
                ${OpenCV_LIBRARY_DIRS} 
                ${CERES_LIBRARY_DIRS} 
                ${GLOG_LIBRARY_DIRS} 
                ${GFLAGS_LIBRARY_DIRS}
                )

file(GLOB_RECURSE SRCS src/*.cpp src/planecalib/*.cpp src/planecalib_ui/*.cpp)
set(OPENCV_LIBRARIES opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_ml opencv_video opencv_features2d opencv_calib3d opencv_objdetect opencv_flann)

link_libraries(selfcalib ${OPENCV_LIBRARIES} 
                        ${CERES_LIBRARIES} 
                        ${GLOG_LIBRARIES} 
                        ${GFLAGS_LIBRARIES} 
                        )

add_executable(selfcalib ${SRCS})
target_link_libraries (selfcalib)
