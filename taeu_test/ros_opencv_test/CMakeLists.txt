cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv_test)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
)
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge roscpp
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(cv_bridge_test src/ros_opencv_test.cpp)
target_link_libraries(cv_bridge_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})