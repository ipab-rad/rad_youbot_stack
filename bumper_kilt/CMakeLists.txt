cmake_minimum_required(VERSION 2.8.3)
project(bumper_kilt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bumper_kilt
  src/bumper_kilt.cpp
)

target_link_libraries(bumper_kilt
  ${catkin_LIBRARIES}
)
