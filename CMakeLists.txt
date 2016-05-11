cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(RS_scanner)


#################################### Locate PCL ############################################

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(LIBS ${LIBS} ${PCL_LIBRARIES})


#################################### Locate OpenCV ############################################

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})

#################################### Locate realsense #####################################
include_directories("/home/aburbanol/librealsense/include")
link_directories("/usr/local/lib")
set(LIBS ${LIBS} realsense)

link_directories( ${realsense_LIBRARY})
include_directories( ${realsense_INCLUDE_DIR})


add_executable (RS_scanner main.cpp cloud_utils.h cloud_utils.cpp)
target_link_libraries (RS_scanner ${LIBS} )

