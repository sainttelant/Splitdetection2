#CMakeLists for src directory
project(adasServer)
set(${PROJECT_NAME}_SRCS
    main.cpp
    adas_em.cpp
)
add_subdirectory(abdObj)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

# tianjia opencv lib path
target_link_libraries(${PROJECT_NAME}
 ${OpenCV_LIBS}
  eCAL::core
  abdObj
)
