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
Clone the current repository not in the src file. The qcontrol_defs package have to be build before the other ROS package
```sh
cd ~/catkin_ws
git clone https://github.com/u-t-autonomous/PX4_ROS_packages.git
mv PX4_ROS_packages/qcontrol_defs src/
```

Then clone the following three catkin_packages and build
```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/joystick_drivers.git
git clone git clone https://github.com/ethz-asl/vicon_bridge
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