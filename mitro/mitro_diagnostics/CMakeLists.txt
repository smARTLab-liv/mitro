cmake_minimum_required(VERSION 2.8.3)
project(mitro_diagnostics)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    std_msgs
    message_generation
    diagnostic_msgs
)

add_message_files(
  FILES
  BatteryStatus.msg
  NetworkStatus.msg
  SystemStatus.msg
  SysInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  diagnostic_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs diagnostic_msgs message_runtime
)
