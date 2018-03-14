#include "PosControl.h"

//Sets initial errors to zero
void initializePID(PID_3DOF &PID){
	Eigen::Vector3d zeros = Eigen::Vector3d::Zero();

	PID.e_prop = zeros;
	PID.e_deriv = zeros;
	PID.e_integ = zeros;
}

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF &PID){
	PID.e_integ = Eigen::Vector3d::Zero();
}

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF &PID, 
	                       const Eigen::Vector3d &K_p, 
	                       const Eigen::Vector3d &K_i, 
	                       const Eigen::Vector3d &K_d, 
	                       const Eigen::Vector3d &maxInteg){
	PID.K_p = K_p;
	PID.K_d = K_d;
	PID.K_i = K_i;
	PID.maxInteg = maxInteg;
}

//Update all errors
void updateErrorPID(PID_3DOF &PID, 
	                const Eigen::Vector3d &feedForward, 
	                const Eigen::Vector3d &e_prop, 
	                const Eigen::Vector3d &e_deriv, 
	                float dt){
	PID.feedForward = feedForward;
	PID.e_prop = e_prop;
	PID.e_deriv = e_deriv;
	PID.e_integ = PID.e_integ+(e_prop*dt); //e_integ = e_integ + e_prop*dt

	//Saturate integral error between lower and upper bounds
	for (int i = 0; i < 3; i++){
		PID.e_integ(i) = saturate(PID.e_integ(i), 
			                     -PID.maxInteg(i), 
			                      PID.maxInteg(i));
	}
}

//Calculate output of PID
Eigen::Vector3d outputPID(const PID_3DOF &PID){
	Eigen::Vector3d PID_out;
	PID_out =  PID.feedForward + 
			   PID.e_prop.cwiseProduct(PID.K_p) + 
			   PID.e_deriv.cwiseProduct(PID.K_d) + 
			   PID.e_integ.cwiseProduct(PID.K_i);		
	return PID_out;
}

//Initialize parameters for position control
void initializePosControlParam(PosControlParam &Param,
	                           double mass, double gz,
	                           double thrustRatio){
	Param.mass = mass;
	Param.gz = gz;
	Param.thrustRatio = thrustRatio;

}

void readROSparameterServer(PID_3DOF &PID, PosControlParam &Param , ros::NodeHandle &nh){

  double mass, gz, thrustRatio;
  Eigen::Vector3d Kp, Ki, Kd, maxInteg; 

  //Get system properties
  if (! nh.getParam("mass", mass)){
  	mass = 1.535;
  }
  if( ! nh.getParam("gz", gz)){
  	gz = 9.81;
  }
  if(! nh.getParam("thrustRatio", thrustRatio)){
  	thrustRatio = 1.7;
  }

  initializePosControlParam(Param, mass, gz, thrustRatio);

  //Get controller parameters
  if( ! nh.getParam("kpx", Kp[0])){
  	Kp[0] = 10;
  }
  if ( !nh.getParam("kpy", Kp[1])){
  	Kp[1] = 10;
  }
  if ( !nh.getParam("kpz", Kp[2])){
  	Kp[2] = 10;
  }
  if ( !nh.getParam("kvx", Kd[0])){
  	Kd[0] = 5;
  }
  if( !nh.getParam("kvy", Kd[1])){
  	Kd[1] = 5;
  }
  if( !nh.getParam("kvz", Kd[2])){
  	Kd[2] = 7.5;
  }
  if( !nh.getParam("kix", Ki[0])){
  	Ki[0] =0.0;
  }
  if ( !nh.getParam("kiy", Ki[1])){
  	Ki[1] = 0.0;
  }
  if( !nh.getParam("kiz", Ki[2])){
  	Ki[2] = 0.0;
  }
  if( !nh.getParam("maxInteg_x", maxInteg[0])){
  	maxInteg[0] = 0.0;
  }
  if( !nh.getParam("maxInteg_y", maxInteg[1])){
  	maxInteg[1] = 0.0;
  }
  if( !nh.getParam("maxInteg_z", maxInteg[2])){
  	maxInteg[2] = 0.0;
  }

  updateControlParamPID(PID, Kp, Ki, Kd, maxInteg);

  //Print all parameter values
  ROS_INFO("mass: %f\tgz: %f\tthrustRatio: %f", mass, gz, thrustRatio);
  ROS_INFO("Kp: %f,\t%f,\t%f", Kp[0], Kp[1], Kp[2]);
  ROS_INFO("Kd: %f,\t%f,\t%f", Kd[0], Kd[1], Kd[2]);
  ROS_INFO("Ki: %f,\t%f,\t%f", Ki[0], Ki[1], Ki[2]);
  ROS_INFO("maxInteg: %f,\t%f,\t%f\n", maxInteg[0], maxInteg[1], maxInteg[2]);

}

/*void PosController(const nav_msgs::Odometry &Odom,
	               const PVA_structure &PVA_ref,
	               const PosControlParam &Param,
	               double dt,
	               PID_3DOF &PosPID,
	               geometry_msgs::PoseStamped &PoseRef,
	               std_msgs::Float64 &refThrust){

	//Local variables
	double m = Param.mass;
	double gz = Param.gz;
	double maxThrust = Param.thrustRatio*m*gz;

	double yawRef;							//Desired yaw
	Eigen::Vector3d e_Pos, e_Vel; 			//error in position and velocity
	Eigen::Vector3d PosRef, VelRef, AccRef;	//Desired PVA
	Eigen::Vector3d Pos, Vel;				//Current PV
	Eigen::Vector3d feedForward;			//Feedforward vector
	Eigen::Vector3d Fdes;					//Desired force in body frame
	Eigen::Vector3d z_w, z_b;				//z direction in world and body frames 
	Eigen::Matrix3d Rbw;					//Rotation from body to world
	Eigen::Matrix3d Rdes;					//Desired rotation
	Eigen::Vector3d z_bdes, x_cdes, y_bdes, x_bdes;

	//Desired states
	PosRef << PVA_ref.Pos.pose.position.x,
	          PVA_ref.Pos.pose.position.y,
	          PVA_ref.Pos.pose.position.z;
	VelRef << PVA_ref.Vel.twist.linear.x,
	          PVA_ref.Vel.twist.linear.y,
	          PVA_ref.Vel.twist.linear.z;
	AccRef << PVA_ref.Acc.accel.linear.x,
	          PVA_ref.Acc.accel.linear.y,
	          PVA_ref.Acc.accel.linear.z;
	yawRef = getHeadingFromQuat(PVA_ref.Pos.pose.orientation);

	//Current states
	Pos << Odom.pose.pose.position.x,
	       Odom.pose.pose.position.y,
	       Odom.pose.pose.position.z;
	Vel << Odom.twist.twist.linear.x,
	       Odom.twist.twist.linear.y,
	       Odom.twist.twist.linear.z;

	//Rotation matrix and z frame vectors
	Rbw = quat2rot(Odom.pose.pose.orientation);
	z_w << 0, 0, 1;
	z_b = Rbw*z_w;

	//Calculate errors (Obs: Odom might be in body or inertial frame)
	e_Pos = PosRef - Pos;
	std::string frameId = Odom.header.frame_id;
  	std::string childFrameId = Odom.child_frame_id;
	if(frameId.compare(childFrameId) == 0){
		e_Vel = VelRef - Vel;
	}
	else{
		e_Vel = VelRef - Rbw*Vel;
	}
	
	//Translational controller
	feedForward = m*gz*z_w + m*AccRef;
	updateErrorPID(PosPID, feedForward, e_Pos, e_Vel, dt);
	Fdes = outputPID(PosPID);

	//Desired thrust in body frame
	refThrust.data = min(max(Fdes.dot(z_b), 0.0),maxThrust)/maxThrust;

	//Find desired attitude from desired force and yaw angle
	z_bdes = normalizeVector3d(Fdes);
	x_cdes << cos(yawRef), sin(yawRef), 0;
	y_bdes = normalizeVector3d(z_bdes.cross(x_cdes));
	x_bdes = y_bdes.cross(z_bdes);
	Rdes << x_bdes, y_bdes, z_bdes;

	//Set references
	PoseRef.pose.position = PVA_ref.Pos.pose.position;
	PoseRef.pose.orientation = rot2quat(Rdes);
}*/