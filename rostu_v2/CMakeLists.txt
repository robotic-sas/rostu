cmake_minimum_required(VERSION 2.8.3)
project(rostu_v2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  roslib
  rospy
  tf
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  BallCoor.msg
  Dribling.msg
  Kicker.msg
)

generate_messages(

)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rostu_navigation src/rostu_navigation.cpp)
add_executable(rostu_camera_read src/rostu_camera_read.cpp)
add_executable(rostu_line_tracking src/rostu_line_tracking.cpp)
add_executable(rostu_ball_tracking src/rostu_ball_tracking.cpp)
add_executable(rostu_v2_calibration src/rostu_calibration.cpp)
add_executable(rostu_vel_mul src/vel_mul.cpp)
add_executable(rostu_scan_adj src/scanning_matching_adjustment.cpp)

target_link_libraries(rostu_navigation
  ${catkin_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_camera_read
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(rostu_line_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_ball_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_v2_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_vel_mul
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(rostu_scan_adj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
