cmake_minimum_required(VERSION 2.8.3)
project(hiro_card_demo_resumption_management)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  behaviortree_ros
  roseus_bt
  std_msgs
  actionlib_msgs
)
find_package(fmt)

add_service_files(
  FILES
  HasClearRequest.srv
  HasGoalRequest.srv
)

add_action_files(
  FILES
  DETECT.action
  PICK_AND_PLACE.action
  RETRIEVE.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  actionlib_msgs
  DEPENDS fmt
)


include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(${roseus_bt_SOURCE_PREFIX}/include/rosbridgecpp rosbridgecpp)

add_executable(hiro_card_demo src/hiro_card_demo.cpp)
add_dependencies(hiro_card_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiro_card_demo ${catkin_LIBRARIES} rosbridgecpp fmt::fmt)


