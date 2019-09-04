cmake_minimum_required(VERSION 2.8.3)
project(rostu_simulation)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  rospy
  roscpp
  roslib
  rviz
  move_base_msgs
  tf
  actionlib
)

find_package(OpenCV REQUIRED)
find_package(gazebo)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rostu_simulation
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
)

add_executable(field src/field.cpp)
add_executable(rostu_simulation_scan_adj src/scanning_matching_adjustment.cpp)
add_executable(rostu_simulation_odom_adj src/odom_adj.cpp)
add_executable(rostu_simulation_robot_speed src/robot_speed.cpp)

target_link_libraries(field
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${gazebo_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_simulation_scan_adj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${gazebo_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_simulation_odom_adj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_simulation_robot_speed
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
