cmake_minimum_required(VERSION 2.8.3)
project(alice_foot_step_planner)

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
#  message_generation
  robotis_controller_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

#add_message_files(
#   FILES
#   FootStepCommand.msg
#   Step2D.msg
#   Step2DArray.msg
#)

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs robotis_controller_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(alice_foot_step_planner
   src/alice_foot_step_planner.cpp
   src/main.cpp
)

add_dependencies(alice_foot_step_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_foot_step_planner
   yaml-cpp
   ${catkin_LIBRARIES}
)