cmake_minimum_required(VERSION 2.8.3)
project(min_snap)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   ConstraintValue.msg
   ConstraintAxis.msg
   WayPoints.msg
   Trajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY srv
   FILES
   PathPlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SRCS
  src/min_snap.cpp
  src/ap.cpp
  src/alglibmisc.cpp
  src/alglibinternal.cpp
  src/linalg.cpp
  src/solvers.cpp
  src/optimization.cpp
  src/min_snap_node.cpp
  )

add_executable(${PROJECT_NAME}_node ${SRCS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)