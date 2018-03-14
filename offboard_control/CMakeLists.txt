cmake_minimum_required(VERSION 2.8.3)
project(offboard_control)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  tf
  qcontrol_defs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs sensor_msgs qcontrol_defs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
#add_executable(offboard_control_node src/offboard_control.cpp)

add_executable(copter_visualization_node
            src/copter_visualization.cpp
            )
add_executable(offboard_control_node 
            src/offboard_control.cpp
            src/PosControl/PosControl.cpp
            src/utils.cpp)

add_executable(snap_joy_node 
  src/snap_joy.cpp
  src/utils.cpp)

add_executable(vicon_remap_node src/vicon_remap.cpp)
#add_executable(gazebo_vicon_remap_node src/gazebo_vicon_remap.cpp)

add_dependencies(offboard_control_node ${catkin_EXPORTED_TARGETS})
add_dependencies(snap_joy_node ${catkin_EXPORTED_TARGETS})
add_dependencies(copter_visualization_node ${catkin_EXPORTED_TARGETS})
add_dependencies(vicon_remap_node ${catkin_EXPORTED_TARGETS})
#add_dependencies(gazebo_vicon_remap_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(offboard_control_node
  ${catkin_LIBRARIES}
)

target_link_libraries(snap_joy_node
  ${catkin_LIBRARIES}
)

target_link_libraries(copter_visualization_node
  ${catkin_LIBRARIES}
)

target_link_libraries(vicon_remap_node
  ${catkin_LIBRARIES}
)
#target_link_libraries(gazebo_vicon_remap_node
#   ${catkin_LIBRARIES}
#)
