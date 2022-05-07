cmake_minimum_required(VERSION 2.8.3)
project(rov_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  gamepad.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS src/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
     CATKIN_DEPENDS message_runtime
)