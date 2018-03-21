cmake_minimum_required(VERSION 2.8.3)
project(adas_vision_mmw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  cv_bridge
  image_transport
)

find_package(OpenGL REQUIRED)
list( APPEND SUMMARY_DEBUG_LIBS ${OPENGL_gl_LIBRARY} )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/guidedfilter.cpp
)


add_executable(main src/main.cpp src/guidedfilter.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  
add_dependencies(main adas_vision_mmw_gencpp)

add_executable(esr src/esr.cpp)
target_link_libraries(esr ${catkin_LIBRARIES})    
