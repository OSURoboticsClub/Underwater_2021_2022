cmake_minimum_required(VERSION 2.8.3)
project(rov_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_package(
     CATKIN_DEPENDS message_runtime
)

add_message_files(
  FILES
  gamepad.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)