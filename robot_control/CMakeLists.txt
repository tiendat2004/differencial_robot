cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  joint_state_publisher
  robot_state_publisher
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_publisher src/odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
#############
## Install ##
#############

# Thêm các phần cài đặt nếu cần

