cmake_minimum_required(VERSION 2.8.3)
project(youbot_moveit_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface tf pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES youbot_moveit_manipulation
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    tf
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(planning)
add_subdirectory(kinematics)
add_subdirectory(demos)