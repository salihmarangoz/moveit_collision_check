cmake_minimum_required(VERSION 3.0.2)
project(moveit_collision_check)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  tf2_eigen
  visualization_msgs
  message_generation
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  CheckCollision.srv
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_collision_check
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(collision_check
  src/collision_check.cpp
)

add_dependencies(collision_check ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(collision_check
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)