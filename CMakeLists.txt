cmake_minimum_required(VERSION 3.5)
# Enable C++11
project(adasServer)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# yi xia zhi chi dbg tiao shi 
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#设置可执行文件与链接库的保存路径
set(EXECUTABLE_OUTPUT_PATH${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH${PROJECT_SOURCE_DIR}/lib)

 
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI

find_package(VisionWorks REQUIRED)
find_package(VisionWorks-NVXIO)
include_directories("${VisionWorks_INCLUDE_DIRS}")
#include_directories("${VisionWorks-NVXIO_INCLUDE_DIRS}")
find_package(Threads REQUIRED)
find_package(eCAL REQUIRED)


find_package(OpenCV REQUIRED)
include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/src/abdObj/
    )

message(STATUS "OpenCV dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_libs:${OpenCV_LIBS}")

add_subdirectory(src)

 

