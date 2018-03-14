cmake_minimum_required(VERSION 3.0)

project(min_snap_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  qcontrol_defs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime qcontrol_defs
)

#find_package(Eigen3 3.3 REQUIRED)

add_subdirectory(src/min_snap min_snap)
set(min_snap_LIBS "min_snap" )


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${min_snap_INCLUDE_DIRS}
)

set(SRCS
	src/min_snap_node.cpp
)


add_executable(${PROJECT_NAME}_node ${SRCS})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${min_snap_LIBS})
