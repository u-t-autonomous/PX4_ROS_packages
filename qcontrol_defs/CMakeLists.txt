cmake_minimum_required(VERSION 2.8.3)
project(qcontrol_defs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
   DIRECTORY msg
   FILES
   PVA.msg
   PosControl.msg
   VelControl.msg
   AccControl.msg
   AttControl.msg
   QuadState.msg
   ConstraintValue.msg
   ConstraintAxis.msg
   Trajectory.msg
   WayPoints.msg
 )

 add_service_files(
  DIRECTORY srv
  FILES
  CommandAction.srv
  PathPlan.srv
  updatePx4param.srv
  SimplePathPlan.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
