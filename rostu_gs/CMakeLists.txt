cmake_minimum_required(VERSION 2.8.3)
project(rostu_gs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rostu_gs
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rostu_referee_listener src/rostu_referee_listener.cpp)

target_link_libraries(rostu_referee_listener
 ${catkin_LIBRARIES}
  yaml-cpp
)
