#CMakeLists for src directory

project(abdObj)
set(${PROJECT_NAME}_SRCS
    FrameDiff.cpp
    ImageAnalysis.cpp
    IOUT.cpp
    SplitIF.cpp
)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries(abdObj
  ${OpenCV_LIBS}
  ${VisionWorks_LIBRARIES}
  ${VisionWorks-NVXIO_LIBRARIES}
 
)



