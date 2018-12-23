cmake_minimum_required(VERSION 2.8.3)
project(hiro_card_demo)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation)

add_message_files(FILES
  Card.msg
  CardArray.msg
)

add_service_files(
  FILES
  Goal.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories()
