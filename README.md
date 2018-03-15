# PX4_ROS_packages
This package provides ROS tools to work with PX4 Simulation on Gazebo and AirSIM, PX4 autopilot integrated with Snapdragon or another board.

# Package installation and dependencies

This manual have been tested on a clean Ubuntu 16.04 LTS installation.

## Common dependencies
```sh
# Basics dependencies
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential gfortran libeigen3-dev genromfs ninja-build -y

# Required python packages for PX4
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial
```

## Minimum snap dependencies
SuiteSparse, cholmod are needed in order to accelerate matrix operations and applying parrallelism.
GSL is also need.
```sh
sudo apt-get install libgsl-dev libsuitesparse-dev
```

## Joystick dependencies
```sh
sudo apt-get install libspnav-dev libbluetooth-dev libcwiid-dev libusb-dev
```

## ROS dependencies
Install ROS for [Ubuntu 16.04](http://wiki.ros.org/kinetic/Installation/Ubuntu).

You will also need MAVROS installed on your computer. if not, follow this :
```sh
# For Ros Kinteic (Ubuntu 16.04)
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

Create and initialize ROS workspace if needed
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

If already not done, add the following lines at the end of your .bashrc file:
```sh
export ROS_MASTER_URI=http://192.168.1.XX:11311
export ROS_IP=192.168.1.XX
```
where the first 'XX' is the master IP. The second is your own IP.

## Compilation of the ROS packages
Clone and build the following two catkin_packages
```sh
cd ~/catkin_ws/src
# Mandatory to compile snap_joy node
git clone https://github.com/ros-drivers/joystick_drivers.git
# Not mandatory
git clone https://github.com/ethz-asl/vicon_bridge
cd ..
catkin_make
source devel/setup.sh
```

Clone the current repository not in the src file. The qcontrol_defs package have to be build before the other packages
```sh
cd ~/catkin_ws
git clone https://github.com/u-t-autonomous/PX4_ROS_packages.git
cd PX4_ROS_packages
git checkout v2.0_08_2017
git submodule update --init --recursive
cd ..
cp -r PX4_ROS_packages/qcontrol_defs src/
catkin_make
source devel/setup.sh
```

Finally build the rest of the PX4_ROS_packages nodes
```sh
cd ~/catkin_ws/
cp -r PX4_ROS_packages/* src/
catkin_make
source devel/setup.sh
# You may want to delete or not the cloned code
rm -rf PX4_ROS_packages
```
# offboard_control package
This sub package handles the low level interaction between the offboard computer and PX4 on the quad. Basically you can send using this package velocity, position , attitude setpoint to the quad. You can also request takeoff, land , switching between mode request etc...


# Reactive controller test
