cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  quadrotor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(include
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS} 
)

add_message_files(
  FILES
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  quadrotor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS roscpp rospy nav_msgs quadrotor_msgs message_runtime
  DEPENDS Eigen3
)

add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
   OsqpEigen::OsqpEigen
)
