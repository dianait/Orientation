cmake_minimum_required(VERSION 2.8.3)
project(jinko_navigation_service)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  jinko_service_msg
  smach
  smach_ros
  actionlib
  move_base_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
 CATKIN_DEPENDS rospy
 jinko_service_msg

)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

