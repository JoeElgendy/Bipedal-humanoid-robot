cmake_minimum_required(VERSION 2.8.3)
project(humanoid_kinematics)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core
  moveit_kinematics
  moveit_commander
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_ros_manipulation
  kdl_parser
  std_msgs
  geometric_shapes
)


catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME} 
   CATKIN_DEPENDS roscpp rospy 
   DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)


add_executable(humanoid_kinematics src/humanoid_kinematics.cpp )
target_link_libraries(humanoid_kinematics ${catkin_LIBRARIES})





