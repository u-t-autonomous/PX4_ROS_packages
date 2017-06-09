# PX4_ROS_packages
This package provides tools to work with snapdragon over ROS.
Before being able to use all these ROS nodes, these dependencies have to be installed.

# Package installation

Install the dependencies
```sh
sudo apt-get install libspnav-dev libbluetooth-dev libcwiid-dev
```

Install ROS using [this](http://wiki.ros.org/indigo/Installation/Ubuntu). This is the indigo version. Feel freee to install the version of your choice.

Create ROS workspace if needed
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```
If already not done, add the following lines to the end of the .bashrc file:
```sh
export ROS_MASTER_URI=http://192.168.1.XX:11311
export ROS_IP=192.168.1.XX
```
where 'XX' is your own IP.

Clone the current repository not in the src file. The qcontrol_defs package have to be build before the other ROS package
```sh
cd ~/catkin_ws
git clone https://github.com/u-t-autonomous/PX4_ROS_packages.git
mv PX4_ROS_packages/qcontrol_defs src/
```

Then clone the following two catkin_packages and build
```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/joystick_drivers.git
git clone https://github.com/ethz-asl/vicon_bridge
cd ..
catkin_make
source devel/setup.sh
```

Finally build the rest of the PX4_ROS_packages nodes
```sh
cd ~/catkin_ws/
mv PX4_ROS_packages/* src/
catkin_make
source devel/setup.sh
```
# offboard_control package
This sub package handles the low level interaction between the offboard computer and PX4 on the quad. Basically you can send using this package velocity, position , attitude setpoint to the quad. You can also request takeoff, land , switching between mode request etc...

Before launching this node, be sure the PX4 is launch on quad side and that vicon is already tracking the quad.
```sh
ssh linaro@192.168.1.XX
sudo ./px4 mainapp.config
```
where 'XX' is the IP of your snapdragon and the password for connection is `linaro`.

Now launch the node that interacts with the quad
```sh
roslaunch offboard_control quadX_offboard.launch
```
where X is the vicon number you assigned to the quad. (More mprovement have to be taken her !!! Documentation will change)

When offboard_node is launched, the quad can be used via any other ros node. 

###	A C++ example can be find [HERE(joy control of the quad)](https://github.com/u-t-autonomous/PX4_ROS_packages/blob/master/offboard_control/src/snap_joy.cpp)

### A Python example can be find here [HERE(global methods for taking off,landing, switch to poctl, speedctl ...)](https://github.com/u-t-autonomous/PX4_ROS_packages/blob/master/reactive_test/src/system_node.py)

An example can be controlling the quad using a joy. Just launch the predefined launch file
```sh
roslaunch offboard_control quadX_joy.launch
```
where X is the name of the quad in vicon 

example of launch file can be modified for a specific quad name or quad IP or udp mavros port. Just look at the ones in this repository if looking for example. Parameter are pretty self explanatory.

# Reactive controller test
