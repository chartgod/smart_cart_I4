cmake_minimum_required(VERSION 3.0.2)
project(kobuki_pose_topic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  move_base_msgs
)

add_message_files(
  FILES
  MsgTutorial.msg
  #MsgTestbotPose.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs move_base_msgs)

catkin_package(
  LIBRARIES kobuki_pose_topic
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(topic_publisher src/topic_publisher.cpp)
add_dependencies(topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_publisher ${catkin_LIBRARIES})


# add_executable(topic_publisher src/demo_msg_publisher.cpp)
# add_dependencies(topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(topic_publisher ${catkin_LIBRARIES})


# add_executable(topic_publisher2 src/topic_publisher2.cpp)
# add_dependencies(topic_publisher2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(topic_publisher2 ${catkin_LIBRARIES})

add_executable(topic_subscriber src/topic_subscriber.cpp)
add_dependencies(topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_subscriber ${catkin_LIBRARIES})

# add_executable(testbot_topic_publisher src/testbot_topic_publisher.cpp)
# add_dependencies(testbot_topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(testbot_topic_publisher ${catkin_LIBRARIES})

# add_executable(testbot_topic_subscriber src/testbot_topic_subscriber.cpp)
# add_dependencies(testbot_topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(testbot_topic_subscriber ${catkin_LIBRARIES})