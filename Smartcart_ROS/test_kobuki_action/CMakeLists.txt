cmake_minimum_required(VERSION 3.0.2)
project(test_kobuki_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  roscpp
  std_msgs
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(FILES KobukiMsg.action)
add_message_files(FILES KobukiMsg.msg)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
 LIBRARIES test_kobuki_action
 CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation move_base_msgs roscpp std_msgs
 DEPENDS Boost
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# add_executable(action_server src/action_server.cpp)
# add_dependencies(action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(action_server ${catkin_LIBRARIES})

# add_executable(action_client src/action_client.cpp)
# add_dependencies(action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(action_client ${catkin_LIBRARIES})

# add_executable(map_action_client src/map_action_client.cpp)
# add_dependencies(map_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(map_action_client ${catkin_LIBRARIES})

# add_executable(A_B_action_client src/A_B_action_client.cpp)
# add_dependencies(A_B_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(A_B_action_client ${catkin_LIBRARIES})

add_executable(0417_stop_test_action_client src/0417_stop_test_action_client.cpp)
add_dependencies(0417_stop_test_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(0417_stop_test_action_client ${catkin_LIBRARIES})

add_executable(0414_go_to_target_point_test_action_client src/0414_go_to_target_point_test_action_client.cpp)
add_dependencies(0414_go_to_target_point_test_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(0414_go_to_target_point_test_action_client ${catkin_LIBRARIES})

add_executable(0414test_action_server src/0414test_action_server.cpp)
add_dependencies(0414test_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(0414test_action_server ${catkin_LIBRARIES})


