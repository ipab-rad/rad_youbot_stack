cmake_minimum_required(VERSION 2.8.3)
project(youbot_navigation_local)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  costmap_2d
  dynamic_reconfigure
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  costmap_2d
  dynamic_reconfigure
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_layer
  src/simple_layer.cpp
)


add_library(grid_layer
  src/grid_layer.cpp
)

add_library(outer_layer
  src/outer_layer.cpp
)

add_executable(outer_test_pub
  src/outer_test_pub.cpp
)

target_link_libraries(simple_layer
  ${catkin_LIBRARIES}
)

target_link_libraries(outer_layer
  ${catkin_LIBRARIES}
)

target_link_libraries(outer_test_pub
  ${catkin_LIBRARIES}
)
