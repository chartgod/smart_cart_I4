cmake_minimum_required(VERSION 3.0.2)
project(tracking)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  darknet_ros_msgs
)

# find_package( OpenCV REQUIRED )
find_package( OpenCV 4.2.0 REQUIRED )
# find_package( darknet_ros REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)

# add_action_files(FILES KobukiMsg.action)
# add_message_files(FILES KobukiMsg.msg)
add_message_files(FILES TrackingMsg.msg)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)# darknet_ros_msgs)

catkin_package(
 LIBRARIES opencv
 LIBRARIES tracking
 CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation move_base_msgs roscpp std_msgs image_transport
 darknet_ros_msgs
 DEPENDS Boost
)

include_directories( 
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

add_executable(tracking_test src/tracking_test.cpp)
add_dependencies(tracking_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_test ${catkin_LIBRARIES})
target_link_libraries(tracking_test ${OpenCV_LIBRARIES})

add_executable(darknet_ros_tracking_test src/darknet_ros_tracking_test.cpp)
add_dependencies(darknet_ros_tracking_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(darknet_ros_tracking_test ${catkin_LIBRARIES})
target_link_libraries(darknet_ros_tracking_test ${OpenCV_LIBRARIES})

add_executable(cam_test src/cam_test.cpp)
add_dependencies(cam_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_test ${catkin_LIBRARIES})
target_link_libraries(cam_test ${OpenCV_LIBRARIES})

add_executable(cam_test2 src/cam_test2.cpp)
add_dependencies(cam_test2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_test2 ${catkin_LIBRARIES})
target_link_libraries(cam_test2 ${OpenCV_LIBRARIES})

# add_executable(0414test_action_server src/0414test_action_server.cpp)
# add_dependencies(0414test_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(0414test_action_server ${catkin_LIBRARIES})


