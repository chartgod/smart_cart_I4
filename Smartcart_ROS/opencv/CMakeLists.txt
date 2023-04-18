cmake_minimum_required(VERSION 3.0.2)
project(opencv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
)
find_package (OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs
# DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(opencv_pub src/pub.cpp)
target_link_libraries(opencv_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(opencv_sub src/sub.cpp)
target_link_libraries(opencv_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})