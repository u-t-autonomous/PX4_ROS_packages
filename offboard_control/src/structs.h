#ifndef _H_STRUCTS_
#define _H_STRUCTS_

#include "ros/ros.h"
#include <Eigen/Dense>

//Structure for a PID with 3 degrees-of-freedom
struct PID_3DOF{
	Eigen::Vector3d e_prop;
	Eigen::Vector3d e_deriv;
	Eigen::Vector3d e_integ;
	Eigen::Vector3d feedForward;
	Eigen::Vector3d K_p;
	Eigen::Vector3d K_i;
	Eigen::Vector3d K_d;
	Eigen::Vector3d maxInteg;
};

//Quadcopter parameters
struct PosControlParam{
	double mass;
	double gz;
	double thrustRatio;
};

#endif

