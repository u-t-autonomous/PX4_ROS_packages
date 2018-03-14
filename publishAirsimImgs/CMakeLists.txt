cmake_minimum_required(VERSION 2.8.3)

project(airsim_img_publisher)


set (Airlib_addr  $ENV{HOME}/simulation_environment/AirSim_gcc/AirLib)
set (catkin_workspace_path $ENV{HOME}/catkin_ws) 


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# F:DN to be added when airsim communication is desired

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF) #behzad added, need this always
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  cv_bridge
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package( CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy 
  std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Airlib_addr}/include   
    ${Airlib_addr}/include/controllers
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_workspace_path}/src/publishAirsimImgs/include 
    ${catkin_workspace_path}/src/publishAirsimImgs/src
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${Airlib_addr}/lib 
    ${Airlib_addr}/deps/MavLinkCom/lib  
    ${Airlib_addr}/deps/rpclib/lib
    ${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

add_definitions(${PCL_DEFINITIONS})

add_executable(airsim_imgPublisher 
               src/imgPublisher.cpp 
               src/input_sampler.cpp
               src/HelperFunctions/QuatRotEuler.cpp
               src/Callbacks/callbacks.cpp
)

add_dependencies(airsim_imgPublisher ${catkin_EXPORTED_TARGETS})

target_link_libraries(airsim_imgPublisher ${catkin_LIBRARIES} AirLib  MavLinkCom rpc pthread ${OpenCV_LIBS})

##add_dependencies(airsim_imgPublisher airsim_img_publisher_generate_messages_cpp)

