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
echo 'source ~/catkin_ws/devel/setup.sh' >> ~/.bashrc
```

If already not done, add the following lines at the end of your .bashrc file:
```sh
export ROS_MASTER_URI=http://192.168.1.XX:11311
export ROS_IP=192.168.1.XX
```
where the first 'XX' is the master IP. The second is your own IP.

## Compilation of the ROS packages
Clone the current repository in the src file then build it.
```sh
cd ~/catkin_ws/src
git clone https://github.com/u-t-autonomous/PX4_ROS_packages.git
cd PX4_ROS_packages
git checkout v2.0_08_2017
git submodule update --init --recursive
cd joystick_drivers/
git checkout indigo-devel
cd ~/catkin_ws

catkin_make
# If an error like this : jobserver unavailable: using -j1.  Add '+' to parent make rule
# Then catkin_make again it should work the second time.
catkin_make

source devel/setup.sh
```


NOTE :  Only For simulation with Airsim, the package publishAirsimImgs has to be allow to build executable. This is done by setting the [Airlib_addr](https://github.com/u-t-autonomous/PX4_ROS_packages/blob/41124332bb76a0ffeebaaef0c52d1800b0c9eaf2/publishAirsimImgs/CMakeLists.txt#L6) with the appropriate path to Airlib source file. Finally uncomment [these lines](https://github.com/u-t-autonomous/PX4_ROS_packages/blob/41124332bb76a0ffeebaaef0c52d1800b0c9eaf2/publishAirsimImgs/CMakeLists.txt#L61-L74) and catkin_make again your catkin workspace.

Clone and build the following package if you want to be able to communicate with VICON
```sh
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/vicon_bridge
cd ..
catkin_make
source devel/setup.sh
```

# offboard_control package
This sub package handles the low level interaction between the offboard computer and PX4 on the quad. Basically you can send using this package velocity, position , attitude setpoint to the quad. You can also do take off, land , switching between mode request etc...


# Reactive controller test
