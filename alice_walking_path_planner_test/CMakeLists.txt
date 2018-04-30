cmake_minimum_required(VERSION 2.8.3)
project(alice_walking_path_planner_test)
################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  std_msgs
  geometry_msgs
  robotis_math
 # message_generation
  robotis_controller_msgs
  alice_operation_msgs
)
find_package(Eigen REQUIRED)
################################################################################
# Declare ROS messages, services and actions
################################################################################
#add_message_files(
#   FILES
#   WalkingPathPlanner.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################
################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(alice_walking_path_planner_test
   src/main.cpp
)

add_dependencies(alice_walking_path_planner_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_walking_path_planner_test
   yaml-cpp
   ${catkin_LIBRARIES}
)