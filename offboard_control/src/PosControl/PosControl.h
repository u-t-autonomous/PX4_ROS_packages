#ifndef _H_POSCONTROL__
#define _H_POSCONTROL_

#include <math.h>
#include "nav_msgs/Odometry.h"
#include "../structs.h"
#include "../utils.h"

//Sets initial errors to zero
void initializePID(PID_3DOF &PID);

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF &PID);

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF &PID, 
	                       const Eigen::Vector3d &K_p, 
	                       const Eigen::Vector3d &K_i, 
	                       const Eigen::Vector3d &K_d, 
	                       const Eigen::Vector3d &maxInteg);

//Update all errors
void updateErrorPID(PID_3DOF &PID, 
	                const Eigen::Vector3d &feedForward, 
	                const Eigen::Vector3d &e_prop, 
	                const Eigen::Vector3d &e_deriv, 
	                float dt);

//Calculate output of PID
Eigen::Vector3d outputPID(const PID_3DOF &PID);

//Initialize parameters for position control
void initializePosControlParam(PosControlParam &Param,
	                           double mass, double gz,
	                           double thrustRatio);

//Load parameters from ROS parameter server
void readROSparameterServer(PID_3DOF &PID, PosControlParam &Param, ros::NodeHandle &nh);


#endif