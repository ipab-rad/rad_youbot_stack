cmake_minimum_required(VERSION 2.8.3)
project(youbot_navigation_global)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  costmap_2d
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  costmap_2d
  dynamic_reconfigure
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  AgentPrediction.msg
  AgentPredictions.msg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(outer_layer
  src/outer_layer.cpp
)

add_executable(outer_test_pub
  src/outer_test_pub.cpp
)

target_link_libraries(outer_layer
  ${catkin_LIBRARIES}
)

target_link_libraries(outer_test_pub
  ${catkin_LIBRARIES}
)
