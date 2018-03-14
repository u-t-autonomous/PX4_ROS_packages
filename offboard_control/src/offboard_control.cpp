#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/Thrust.h>
// #include <std_msgs/Float64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/ActuatorControl.h>

#include <qcontrol_defs/PosControl.h>
#include <qcontrol_defs/VelControl.h>
#include <qcontrol_defs/AccControl.h>
#include <qcontrol_defs/AttControl.h>
#include <qcontrol_defs/PVA.h>
#include <qcontrol_defs/updatePx4param.h>

#include <qcontrol_defs/CommandAction.h>
#include <qcontrol_defs/QuadState.h>

//#include <qcontrol_defs/updatePx4param.h>
#include "utils.h"
#include "PosControl/PosControl.h"

#include <atomic>
#include <boost/thread.hpp>
#include <iostream>
#include <string>

#define COMMAND_TOPIC									"qcontrol/commands"
#define TUNE_CONTROL_TOPIC								"qcontrol/pos_control_param"
#define SYSTEM_PARAMS_TOPIC								"qcontrol/system_params"				
#define ATT_CONTROL_TOPIC								"qcontrol/att_control"
#define POS_CONTROL_TOPIC								"qcontrol/pos_control"
#define ACC_CONTROL_TOPIC								"qcontrol/acc_control"
#define VEL_CONTROL_TOPIC								"qcontrol/vel_control"
#define PVA_CONTROL_TOPIC								"qcontrol/pva_control"
#define OFFBOARD_INFO									"qcontrol/offboard_info" 

#define MAIN_LOOP_RATE             						100            //main loop rate for getting faster all the subscriber datas
#define ATT_MODE_RATE									50
#define PVA_MODE_RATE									30
#define OFFBOARD_DELAY									0.5
#define OFFBOARD_INFO_RATE 								20

#define OFFBOARD                 						"OFFBOARD"

#define TAKEOFF_Z										0.8		//Take off altitude ---> Altitude where the takeoff is completed
#define TAKEOFF_DETECTED								0.3		//Altitude of takeoff detected
#define TAKEOFF_KP 										1.0 	//Prop coefficient when taking off with velocity control

#define LANDING_Z										0.06		//Altitude for landing to stop
#define LANDING_KP 										1.0		//Prop coefficient when landing with velocity control

#define PI_ 											3.1415
#define DEFAULT_Z_ROVER									0.0

//Some publisher call global variables
ros::Publisher target_att_pub;
ros::Publisher target_thrust_pub;
ros::Publisher target_pos_pub;
ros::Publisher target_vel_pub;
ros::Publisher target_acc_pub;
ros::Publisher offboard_info;

//Some service call global variables
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;

//Global messages variables
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode set_mode_cmd;

//flag indicating the current mode and the current control mode
//Need atomic types since multithreading app

std::atomic<bool> is_offboard(false);
std::atomic<bool> is_att_control(false);
std::atomic<bool> is_pos_control(false);
std::atomic<bool> is_vel_control(false);
std::atomic<bool> is_acc_control(false);
std::atomic<bool> is_pva_control(false);
std::atomic<bool> is_arm (false);
std::atomic<bool> is_takeoff(false);
std::atomic<bool> is_landing(false);
std::atomic<bool> is_landed(true);
std::atomic<bool> takeoff_complete(false);


std::atomic<bool> is_att_control_thread(false);
std::atomic<bool> is_pos_control_thread(false);
std::atomic<bool> is_vel_control_thread(false);
std::atomic<bool> is_acc_control_thread(false);
std::atomic<bool> is_pva_control_thread(false);
std::atomic<bool> is_takeoff_thread(false);
std::atomic<bool> is_landing_thread(false);

//mutex for locking both access to data commands sent
boost::mutex mutex_odom_local;
boost::mutex mutex_att_pva_control;
boost::mutex mutex_pos_params;

//Main values to send like position, velocity and accelearation
qcontrol_defs::AttControl att_control_msg;
qcontrol_defs::PosControl pos_control_msg;
qcontrol_defs::AccControl acc_control_msg;
qcontrol_defs::VelControl vel_control_msg;
qcontrol_defs::PVA pva_control_msg;


//save odometry value
nav_msgs::Odometry current_odom;
double current_yaw ;

//Params for PVA to attitude control
struct PID_3DOF pid;
struct PosControlParam pos_param;

//Long variable for sequence count in header
long att_seq = 0;
long pos_seq = 0;
long acc_seq = 0;
long vel_seq = 0;

//last request time for services call
ros::Time last_request;

//Parameters variables
double landing_z;
double takeoff_z;

double takeoff_Kp;
double landing_Kp;

int att_mode_rate;
int pva_mode_rate;

// Store the type of vehicule (multirotor or rover)
bool is_mc = true;

/*
* Utility function to wait until a thread is stopped
*/
void wait_thread_to_stop(std::atomic<bool> &associate_var,std::atomic<bool> &curr_thread){
	associate_var.store(false);
	ros::Rate m_rate(50);
	while (curr_thread){
		m_rate.sleep();
	}
}

//Typically set the mode to OFFBOARD MODE
void set_current_state(const char *state,float delay_s,bool force){
    std::string s_state = state;
	if(force){
		ros::Rate rate(30);
        set_mode_cmd.request.custom_mode = state;
		while(ros::ok()  && !(set_mode_client.call(set_mode_cmd) && set_mode_cmd.response.mode_sent)){
			ROS_ERROR(" %s  MODE can not be enabled !!!!! Trying Again .....",state);
			rate.sleep();
		}
		s_state=s_state+" MODE is now enabled at %f secs !!!!!";
        ROS_WARN(s_state.c_str(), ros::Time::now().toSec());
        return;
	}
    if ((ros::Time::now()-last_request)>ros::Duration(delay_s) && current_state.mode != s_state){
        set_mode_cmd.request.custom_mode = state;
        ROS_INFO("PREVIOUS MODE WAS %s !!!!!",current_state.mode.c_str());
        if(set_mode_client.call(set_mode_cmd) && set_mode_cmd.response.mode_sent){
        	s_state=s_state+" MODE is now trying to be enabled at %f secs !!!!!";
            ROS_WARN(s_state.c_str(), ros::Time::now().toSec());
        } else {
        	s_state= s_state+" MODE can not be enabled at %f secs !!!!! Trying Again !!!!!";
        	ROS_ERROR(s_state.c_str(), ros::Time::now().toSec());
        }
        last_request = ros::Time::now();
    }
}

//send and arm request and doesn't stop the function until it has finished
void set_arming(const bool is_arm_){
    arm_cmd.request.value = is_arm_;
    ros::Rate m_rate(10);
    while (ros::ok() && !(arming_client.call(arm_cmd) && arm_cmd.response.success)){
        m_rate.sleep();
    }
    if(!is_arm_){
    	is_offboard.store(false);
    	is_takeoff.store(false);
    	is_landing.store(false);
    	is_pos_control.store(false);
    	is_att_control.store(false);
    	is_vel_control.store(false);
    	is_acc_control.store(false);
    	is_pva_control.store(false);
    }else{
    	is_offboard.store(true);
    }
    ROS_WARN("MOTORS ARE NOW %s !!!!!",is_arm_?"ARMED":"DISARMED");
}

/*
* Sending attitude target point to the vehicle throught mavros
* TODO security check when last message is too old --> can causes a bad crashe || Or maybe used safety area ...
*/
void send_attitude_data(){
	is_offboard.store(true);
	is_att_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);
	wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);

	ROS_WARN("ATTITUDE CONTROL ---> SOURCE FROM %s ", is_pva_control_thread ? "PVA SetPoint !!!" : "ATT_SETPOINT !!!");

	mutex_odom_local.lock();
	geometry_msgs::Vector3 RPY = quat2rpy(current_odom.pose.pose.orientation);

	mutex_att_pva_control.lock();
	att_control_msg.roll = RPY.x;
	att_control_msg.pitch = RPY.y;
	att_control_msg.yaw = RPY.z;
	att_control_msg.thrust = is_landed ? 0.0 : 0.5;
	mutex_att_pva_control.unlock();

	ROS_INFO("INITIAL X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z, current_yaw);
	mutex_odom_local.unlock();

	ros::Rate att_rate(att_mode_rate);

	geometry_msgs::PoseStamped att_msg;
	// std_msgs::Float64  thrust_msg;
	mavros_msgs::Thrust thrust_msg;

	while(ros::ok() && is_offboard && is_att_control){

		geometry_msgs::Vector3 rpy;
		double throttle;

		mutex_att_pva_control.lock();

		rpy.y = att_control_msg.pitch;
		rpy.z = att_control_msg.yaw;
		rpy.x = att_control_msg.roll;
		throttle = att_control_msg.thrust;

		mutex_att_pva_control.unlock();

		mutex_odom_local.lock();
		att_msg.pose.position = current_odom.pose.pose.position;
		mutex_odom_local.unlock();

		att_msg.header.seq = att_seq;
		att_msg.header.frame_id="fcu";
		att_msg.pose.orientation = rpy2quat(rpy);

		thrust_msg. header.seq = att_seq;
		thrust_msg.header.frame_id = "fcu";
		// thrust_msg.data = throttle; 
		thrust_msg.thrust = is_mc ? (throttle>= 0 ? throttle : 0) : throttle;

		att_msg.header.stamp = ros::Time::now();
		thrust_msg.header.stamp = ros::Time::now();


		//Publish the message then sleep
		target_att_pub.publish(att_msg);
		target_thrust_pub.publish(thrust_msg);

		att_seq++;
		att_rate.sleep();
	}

	is_att_control.store(false);
	is_att_control_thread.store(false);
	ROS_WARN("ATTITUDE CONTROL ---> SOURCE FROM %s  STOPPED !!!", is_pva_control_thread ? "PVA SetPoint" : "ATT_SETPOINT");
}


/*
* Sending Velocity target point to the vehicle throught mavros
* This more secured in case of data lost transmission over wifi
*/
void send_velocity_data(){
	is_offboard.store(true);
	is_vel_control_thread.store(true);
	//wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	//wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);
	wait_thread_to_stop(is_att_control,is_att_control_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);
	wait_thread_to_stop(is_pva_control,is_pva_control_thread);

	ROS_WARN("VELOCITY CONTROL STARTED !!! ");

	mutex_odom_local.lock();

	mutex_att_pva_control.lock();
	vel_control_msg.vel = current_odom.twist.twist.linear;
	vel_control_msg.yaw_rate = 0.0;
	vel_control_msg.is_body_frame = false;
	mutex_att_pva_control.unlock();

	ROS_INFO("INITIAL VX , VY , VZ , YAW : %f , %f , %f , %f ", current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y , current_odom.twist.twist.linear.z , current_yaw);
	mutex_odom_local.unlock();

	ros::Rate pva_rate(pva_mode_rate);

	geometry_msgs::TwistStamped vel_msg;
	geometry_msgs::Vector3 vel;
	bool is_body_frame = false;
	double yaw_rate = 0.0;

	while(ros::ok() && is_offboard && is_vel_control){

		mutex_att_pva_control.lock();
		vel = vel_control_msg.vel;
		yaw_rate = vel_control_msg.yaw_rate;
		is_body_frame = vel_control_msg.is_body_frame;
		mutex_att_pva_control.unlock();

		vel_msg.header.seq = vel_seq;
		vel_msg.header.frame_id="fcu";
		mutex_odom_local.lock();
		vel_msg.twist.linear = is_body_frame ?  vel_body_frame(current_yaw,vel) : vel;
		mutex_odom_local.unlock();
		vel_msg.twist.angular.z = yaw_rate;
		vel_msg.twist.angular.y = 0.0;
		vel_msg.twist.angular.x = 0.0;
		if (! is_mc)
			vel_msg.twist.linear.z = 0.0;

		vel_msg.header.stamp = ros::Time::now();

		//Publish the message then sleep
		target_vel_pub.publish(vel_msg);

		vel_seq++;
		pva_rate.sleep();
	}

	is_vel_control.store(false);
	is_vel_control_thread.store(false);
	ROS_WARN("VELOCITY CONTROL STOPPED !!! ");
}

/*
* Sending Acceleration target point to the vehicle throught mavros
* This more secured than attitude control in case of data lost transmission over wifi
*/
void send_acceleration_data(){
	is_offboard.store(true);
	is_acc_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);
	wait_thread_to_stop(is_att_control,is_att_control_thread);
	wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_pva_control,is_pva_control_thread);

	ROS_WARN("ACCELERATION CONTROL STARTED !!! ");

	mutex_odom_local.lock();
	ROS_INFO("INITIAL X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z, current_yaw);
	mutex_odom_local.unlock();

	ros::Rate pva_rate(pva_mode_rate);

	if ( is_mc ){
		geometry_msgs::Vector3Stamped acc_msg;
		geometry_msgs::Vector3 acc;

		while(ros::ok() && is_offboard && is_acc_control){

			mutex_att_pva_control.lock();
			acc = acc_control_msg.acc;
			mutex_att_pva_control.unlock();

			acc_msg.header.seq = acc_seq;
			acc_msg.header.frame_id="fcu";
			acc_msg.vector = acc;

			acc_msg.header.stamp = ros::Time::now();

			//Publish the message then sleep
			target_acc_pub.publish(acc_msg);

			acc_seq++;
			pva_rate.sleep();
		}
	}else {
		mavros_msgs::ActuatorControl acc_msg;
		geometry_msgs::Vector3 acc;
		while (ros::ok() && is_offboard && is_acc_control){
			
			mutex_att_pva_control.lock();
			acc = acc_control_msg.acc;
			mutex_att_pva_control.unlock();

			acc_msg.header.seq = acc_seq;
			acc_msg.header.frame_id="fcu";
			acc_msg.controls[0] = 0.0;
			acc_msg.controls[1] = 0.0;
			acc_msg.controls[2] = acc.y;
			acc_msg.controls[3] = acc.z;

			acc_msg.header.stamp = ros::Time::now();
			if (acc_control_msg.is_body_frame)
				acc_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_MANUAL_PASSTHROUGH;
			else
				acc_msg.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
			
			//Publish the message then sleep
			target_acc_pub.publish(acc_msg);
			acc_seq++;
			pva_rate.sleep();
		}
	}

	is_acc_control.store(false);
	is_acc_control_thread.store(false);
	ROS_WARN("ACCELERATION CONTROL STOPPED !!! ");
}

/*
* Send position target via setpoint_local
* Sending position  and yaw throught that topic.
*/
void send_target_position(){
	is_offboard.store(true);
	is_pos_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);
	wait_thread_to_stop(is_att_control,is_att_control_thread);
	wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_pva_control,is_pva_control_thread);

	ROS_WARN("POSITION CONTROL STARTED !!! ");

	mutex_odom_local.lock();

	mutex_att_pva_control.lock();
	pos_control_msg.pos = current_odom.pose.pose.position;
	pos_control_msg.yaw = current_yaw;
	pos_control_msg.is_body_frame = false;
	mutex_att_pva_control.unlock();

	ROS_INFO("INITIAL X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z, current_yaw);
	mutex_odom_local.unlock();

	ros::Rate pva_rate(pva_mode_rate);

	geometry_msgs::PoseStamped pos_msg;
	geometry_msgs::Point curr_pos;
	geometry_msgs::Vector3 rpy;
	bool is_body_frame = false;
	rpy.x = 0.0;
	rpy.y = 0.0;
	rpy.z = 0.0;

	while(ros::ok() && is_offboard && is_pos_control){

		mutex_att_pva_control.lock();
		curr_pos = pos_control_msg.pos;
		rpy.z = pos_control_msg.yaw;
		is_body_frame = pos_control_msg.is_body_frame;
		mutex_att_pva_control.unlock();

		pos_msg.pose.position = is_body_frame ?  pos_body_frame(rpy.z,curr_pos) : curr_pos;
		if (! is_mc ){
			pos_msg.pose.position.z = DEFAULT_Z_ROVER;
		} 
		pos_msg.pose.orientation = rpy2quat(rpy);
		pos_msg.header.seq = pos_seq;
		pos_msg.header.frame_id="fcu";
		pos_msg.header.stamp = ros::Time::now();

		target_pos_pub.publish(pos_msg);
		pos_seq++;
		pva_rate.sleep();
	}

	is_pos_control.store(false);
	is_pos_control_thread.store(false);

	ROS_WARN("POSITION CONTROL STOPPED !!! ");

}

/*
* Position Velocity Acceleration control of the quad 
* This feature is based on a 3DOF PID for allowing PVA -> attitude control
* Tunning is needed to have it working well
*/
void send_pva_data(){
	is_offboard.store(true);
	is_pva_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);
	wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);

	ROS_WARN("PVA CONTROL THREAD STARTED ...");

	mutex_att_pva_control.lock();
	pva_control_msg.pos = current_odom.pose.pose.position;
	pva_control_msg.vel = current_odom.twist.twist.linear;
	pva_control_msg.acc = geometry_msgs::Vector3();
	pva_control_msg.yaw = current_yaw;
	mutex_att_pva_control.unlock();
	
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

	ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration dt;
    ros::Rate pva_rate(pva_mode_rate * 3);

	while(ros::ok() && is_offboard && is_pva_control){

		current_time = ros::Time::now();
        dt = current_time - last_time;
        last_time = current_time;

		mutex_pos_params.lock();
		double m = pos_param.mass;
		double gz = pos_param.gz;
		double maxThrust = pos_param.thrustRatio * m * gz;
		PID_3DOF temp_pid = pid;
		mutex_pos_params.unlock();

		//Desired states
		mutex_att_pva_control.lock();
		PosRef << pva_control_msg.pos.x,
		          pva_control_msg.pos.y,
		          pva_control_msg.pos.z;
		VelRef << pva_control_msg.vel.x,
		          pva_control_msg.vel.y,
		          pva_control_msg.vel.z;
		AccRef << pva_control_msg.acc.x,
		          pva_control_msg.acc.y,
		          pva_control_msg.acc.z;
		yawRef = pva_control_msg.yaw;
		mutex_att_pva_control.unlock();

		//Current states
		mutex_odom_local.lock();
		Pos << current_odom.pose.pose.position.x,
		       current_odom.pose.pose.position.y,
		       current_odom.pose.pose.position.z;
		Vel << current_odom.twist.twist.linear.x,
		       current_odom.twist.twist.linear.y,
		       current_odom.twist.twist.linear.z;

		std::string frameId = current_odom.header.frame_id;
	  	std::string childFrameId = current_odom.child_frame_id;
	  	//Rotation matrix
	  	Rbw = quat2rot(current_odom.pose.pose.orientation);
		mutex_odom_local.unlock();

		//z frame vectors
		z_w << 0, 0, 1;
		z_b = Rbw*z_w;

		//Calculate errors (Obs: Odom might be in body or inertial frame)
		e_Pos = PosRef - Pos;
		if(frameId.compare(childFrameId) == 0){
			e_Vel = VelRef - Vel;
		}
		else{
			e_Vel = VelRef - Rbw*Vel;
		}
		
		//Translational controller
		feedForward = m*gz*z_w + m*AccRef;
		updateErrorPID(temp_pid, feedForward, e_Pos, e_Vel, dt.toSec());
		Fdes = outputPID(temp_pid);

		//Desired thrust in body frame
		double thrust_val = std::min(std::max(Fdes.dot(z_b), 0.0),maxThrust)/maxThrust; //max_thrust = 1.0

		//Find desired attitude from desired force and yaw angle
		z_bdes = Fdes.normalized();
		x_cdes << cos(yawRef), sin(yawRef), 0;
		y_bdes = (z_bdes.cross(x_cdes)).normalized();
		x_bdes = y_bdes.cross(z_bdes);
		Rdes << x_bdes, y_bdes, z_bdes;

		/*std::cout << "Pos ref = " << PosRef.transpose() << std::endl;
		std::cout << "Pos = " << Pos.transpose() << std::endl;
		std::cout << "Vel ref = " << VelRef.transpose() << std::endl;
		std::cout << "Vel = " << Vel.transpose() << std::endl;
		std::cout << "Err pos = " << e_Pos.transpose() << std::endl;
		std::cout << "err vel = " << e_Vel.transpose() << std::endl;
		std::cout << "Acc ref = " << AccRef.transpose() << std::endl;
		std::cout << "feedForward = " << feedForward.transpose() << std::endl;
		std::cout << "Fdes =" << Fdes.transpose() << std::endl;*/

		//Set references
		geometry_msgs::Vector3 rpy = quat2rpy(rot2quat(Rdes));
		mutex_att_pva_control.lock();
		att_control_msg.roll = rpy.x;
		att_control_msg.pitch = rpy.y;
		att_control_msg.yaw = rpy.z;
		att_control_msg.thrust = thrust_val;
		mutex_att_pva_control.unlock();

		pva_rate.sleep();
	}

	is_pva_control.store(false);
	is_pva_control_thread.store(false);
	ROS_WARN("PVA CONTROL STOPPED");
	return;
}


/*
* Take off function based on a a P controller ---> Should be sufficient for good quad control
* Velocity control till the desired takeoff altitude
*/
void takeoff(){
	is_offboard.store(true);
	is_takeoff_thread.store(true);
	takeoff_complete.store(false);

	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);
	wait_thread_to_stop(is_att_control,is_att_control_thread);
	//wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_pva_control,is_pva_control_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);

	ROS_WARN("STARTING TAKING OFF THREAD .......");
	set_arming(true);
	nav_msgs::Odometry initial_odom_pose , current_odom_pose;
	double init_rate = 0.0;

	mutex_odom_local.lock();
	initial_odom_pose = current_odom;
	ROS_INFO("INITIAL X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z, current_yaw);
	ROS_INFO(" TARGET X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , takeoff_z , current_yaw);
	mutex_odom_local.unlock();

	ros::Rate kp_rate(30);

	//Set the target takeoff altitude
	initial_odom_pose.pose.pose.position.z = takeoff_z;

	//iterate and apply the P controller for taking off
	while(ros::ok() && is_offboard && is_takeoff){

		mutex_odom_local.lock();
		current_odom_pose = current_odom;
		mutex_odom_local.unlock();

		if(! takeoff_complete && (current_odom_pose.pose.pose.position.z > TAKEOFF_DETECTED)){
			takeoff_complete.store(true);
		}

		mutex_att_pva_control.lock();
		vel_control_msg.vel.x = takeoff_Kp * ( initial_odom_pose.pose.pose.position.x - current_odom_pose.pose.pose.position.x);
		vel_control_msg.vel.y = takeoff_Kp * ( initial_odom_pose.pose.pose.position.y - current_odom_pose.pose.pose.position.y);
		vel_control_msg.vel.z = takeoff_Kp * ( initial_odom_pose.pose.pose.position.z - current_odom_pose.pose.pose.position.z);
		vel_control_msg.yaw_rate = takeoff_Kp * ( init_rate - current_odom_pose.twist.twist.angular.z);
		mutex_att_pva_control.unlock();

		kp_rate.sleep();
	}

	is_takeoff.store(false);
	is_takeoff_thread.store(false);
	takeoff_complete.store(false);

	ROS_WARN("STOPPING TAKING OFF THREAD !!!!!");
}

/*
* Landing function based on a a P controller ---> Should be sufficient for good quad control
* Velocity control till the desired landing altitude
* Then shuting down the motors
*/
void land(){
	is_offboard.store(true);
	is_landing_thread.store(true);

	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_acc_control,is_acc_control_thread);
	wait_thread_to_stop(is_att_control,is_att_control_thread);
	//wait_thread_to_stop(is_vel_control,is_vel_control_thread);
	wait_thread_to_stop(is_pva_control,is_pva_control_thread);
	wait_thread_to_stop(is_pos_control,is_pos_control_thread);

	ROS_WARN("STARTING LANDING MODE THREAD......");

	nav_msgs::Odometry initial_odom_pose , current_odom_pose;
	double init_rate = 0.0;

	mutex_odom_local.lock();
	initial_odom_pose = current_odom;
	ROS_INFO("INITIAL X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z, current_yaw);
	ROS_INFO(" TARGET X , Y , Z , YAW : %f , %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , landing_z , current_yaw);
	mutex_odom_local.unlock();

	//Set the target takeoff altitude
	initial_odom_pose.pose.pose.position.z = 0.0;

	ros::Rate kp_rate(30);
	//iterate and apply the P controller for taking off
	while(ros::ok() && is_offboard && is_landing){

		mutex_odom_local.lock();
		current_odom_pose = current_odom;
		mutex_odom_local.unlock();

		if(current_odom_pose.pose.pose.position.z < landing_z){
			set_arming(false);
			break;
		}

		mutex_att_pva_control.lock();
		vel_control_msg.vel.x = landing_Kp * ( initial_odom_pose.pose.pose.position.x - current_odom_pose.pose.pose.position.x);
		vel_control_msg.vel.y = landing_Kp * ( initial_odom_pose.pose.pose.position.y - current_odom_pose.pose.pose.position.y);
		vel_control_msg.vel.z = landing_Kp * ( initial_odom_pose.pose.pose.position.z - current_odom_pose.pose.pose.position.z);
		vel_control_msg.yaw_rate = landing_Kp * ( init_rate - current_odom_pose.twist.twist.angular.z);
		mutex_att_pva_control.unlock();

		kp_rate.sleep();
	}

	is_landing.store(false);
	is_landing_thread.store(false);
	ROS_WARN("STOPPING LANDING MODE THREAD !!!!!!");
}

//State callback function 
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state= *msg;
	if(! current_state.armed && is_arm){
		is_offboard.store(false);
    	is_takeoff.store(false);
    	is_landing.store(false);
    	is_pos_control.store(false);
    	is_att_control.store(false);
    	is_vel_control.store(false);
    	is_acc_control.store(false);
    	is_pva_control.store(false);
	}
    is_arm.store(current_state.armed);

}

//Position setpoint callback
void pos_callback(const qcontrol_defs::PosControl::ConstPtr& msg){
	if(! is_pos_control ){
		return;
	}
	mutex_att_pva_control.lock();
	pos_control_msg = *msg;
	mutex_att_pva_control.unlock();
}

//Velocity setpoint callback
void vel_callback(const qcontrol_defs::VelControl::ConstPtr& msg){
	if(! is_vel_control || (is_takeoff && is_vel_control) || (is_landing && is_vel_control)){
		return ;
	}
	mutex_att_pva_control.lock();
	vel_control_msg = *msg;
	mutex_att_pva_control.unlock();
}

//Acceleration setpoint callback
void acc_callback(const qcontrol_defs::AccControl::ConstPtr& msg){
	if(! is_acc_control){
		return;
	}
	mutex_att_pva_control.lock();
	acc_control_msg = *msg;
	mutex_att_pva_control.unlock();
}

//Attitude setpoint callback
void att_callback(const qcontrol_defs::AttControl::ConstPtr& msg){
	if( ! is_att_control || (is_att_control && is_pva_control)){
		return;
	}
	mutex_att_pva_control.lock();
	att_control_msg = *msg;
	mutex_att_pva_control.unlock();
}

//PVA setpoint callback
void pva_callback(const qcontrol_defs::PVA::ConstPtr& msg){
	//return ;
	if (!is_pva_control){
		return ;
	}
	mutex_att_pva_control.lock();
	pva_control_msg = *msg;
	mutex_att_pva_control.unlock();
}


//Odometry data callback
void curr_pos_callback(const nav_msgs::Odometry::ConstPtr& msg){
	mutex_odom_local.lock();
	current_odom = *msg;
	current_yaw = getHeadingFromQuat(msg->pose.pose.orientation);
	mutex_odom_local.unlock();
}

//Check from the FCU is the quad is landed or not 
void land_state_callback(const mavros_msgs::ExtendedState::ConstPtr& msg){
	if(msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
		is_landed.store(true);
	}else if(msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR){
		is_landed.store(false);
	}
}

bool updatePosControlParam(qcontrol_defs::updatePx4param::Request &req,
	                       qcontrol_defs::updatePx4param::Response &res){
	//Check for the right number of arguments
	if(req.data.size() != 12){
		ROS_INFO("Wrong number of parameters. You need to send parameters in the following order:\n kpx kpy kpz kvx kvy kvz kix kiy kyz maxInteg_x maxInteg_y maxInteg_z");
		return false;
	}

	//Set the parameters
	Eigen::Vector3d Kp, Ki, Kd, maxInteg;
	Kp << req.data[0], req.data[1], req.data[2];
	Kd << req.data[3], req.data[4], req.data[5];
	Ki << req.data[6], req.data[7], req.data[8];
	maxInteg << req.data[9], req.data[10], req.data[11];

	mutex_pos_params.lock();
	updateControlParamPID(pid, Kp, Ki, Kd, maxInteg);
	mutex_pos_params.unlock();

	ROS_INFO("Kp: %f,\t%f,\t%f", Kp[0], Kp[1], Kp[2]);
	ROS_INFO("Kd: %f,\t%f,\t%f", Kd[0], Kd[1], Kd[2]);
	ROS_INFO("Ki: %f,\t%f,\t%f", Ki[0], Ki[1], Ki[2]);
	ROS_INFO("maxInteg: %f,\t%f,\t%f\n", maxInteg[0], maxInteg[1], maxInteg[2]);
	res.success = true;
	
	return true;
}

//Update mass, thrustRatio and gravity
bool updateSystemParam(qcontrol_defs::updatePx4param::Request &req,
	                   qcontrol_defs::updatePx4param::Response &res){

	//Check for the right number of arguments
	if(req.data.size() != 3){
		ROS_INFO("Wrong number of parameters. You need to send parameters in the following order:\n mass gz thrustRatio");
		return false;
	}
	
	//Set the parameters
	double mass, gz, thrustRatio;
	mass = req.data[0];
	gz = req.data[1];
	thrustRatio = req.data[2];

	mutex_pos_params.lock();
	initializePosControlParam(pos_param, mass, gz, thrustRatio);
	mutex_pos_params.unlock();

	ROS_INFO("mass: %f\tgz: %f\tthrustRatio: %f", mass, gz, thrustRatio);

	res.success = true;

	return true;
}

bool handle_command_action(qcontrol_defs::CommandAction::Request &req, qcontrol_defs::CommandAction::Response &res){

	if(req.arm_motors != qcontrol_defs::CommandAction::Request::ARM_MOTOR_UNDEFINED &&
			(req.arm_motors == qcontrol_defs::CommandAction::Request::ARM_MOTOR_TRUE || 
					req.arm_motors == qcontrol_defs::CommandAction::Request::ARM_MOTOR_FALSE)){
		is_arm.store(req.arm_motors==qcontrol_defs::CommandAction::Request::ARM_MOTOR_TRUE?true:false);
		set_arming(is_arm.load());
	}

	if(req.is_attctl != qcontrol_defs::CommandAction::Request::IS_ATTCTL_UNDEFINED &&
			(req.is_attctl == qcontrol_defs::CommandAction::Request::IS_ATTCTL_TRUE || 
					req.is_attctl == qcontrol_defs::CommandAction::Request::IS_ATTCTL_FALSE)){

		bool old_is_attitude = is_att_control.load();
		bool new_is_attitude = req.is_attctl==qcontrol_defs::CommandAction::Request::IS_ATTCTL_TRUE?true:false;
		if(new_is_attitude){
			wait_thread_to_stop(is_pva_control,is_pva_control_thread);
			is_att_control.store(new_is_attitude);
		}else if(! is_pva_control){
			is_att_control.store(new_is_attitude);
		}

		if(is_att_control && !old_is_attitude){
			boost::thread attitude_command(&send_attitude_data);
		}
	}
	
	if(req.is_posctl != qcontrol_defs::CommandAction::Request::IS_POSCTL_UNDEFINED &&
			(req.is_posctl == qcontrol_defs::CommandAction::Request::IS_POSCTL_TRUE || 
					req.is_posctl == qcontrol_defs::CommandAction::Request::IS_POSCTL_FALSE)){
		bool old_is_posctl = is_pos_control.load();
		is_pos_control.store(req.is_posctl==qcontrol_defs::CommandAction::Request::IS_POSCTL_TRUE ? true:false);
		if(is_pos_control && !old_is_posctl){
			boost::thread pos_command(&send_target_position);
		}
	}

	if(req.is_velctl != qcontrol_defs::CommandAction::Request::IS_VELCTL_UNDEFINED &&
			(req.is_velctl == qcontrol_defs::CommandAction::Request::IS_VELCTL_TRUE || 
					req.is_velctl == qcontrol_defs::CommandAction::Request::IS_VELCTL_FALSE)){
		
		bool old_is_velctl = is_vel_control.load();
		bool new_is_velctl = req.is_velctl==qcontrol_defs::CommandAction::Request::IS_VELCTL_TRUE ? true:false;
		if(new_is_velctl){
			wait_thread_to_stop(is_takeoff , is_takeoff_thread);
			wait_thread_to_stop(is_landing , is_landing_thread);
			is_vel_control.store(new_is_velctl);
		}else if( !is_takeoff  &&  !is_landing){
			is_vel_control.store(new_is_velctl);
		}
		if(is_vel_control && !old_is_velctl){
			boost::thread vel_command(&send_velocity_data);
		}
	}

	if(req.is_accctl != qcontrol_defs::CommandAction::Request::IS_ACCCTL_UNDEFINED &&
			(req.is_accctl == qcontrol_defs::CommandAction::Request::IS_ACCCTL_TRUE || 
					req.is_accctl == qcontrol_defs::CommandAction::Request::IS_ACCCTL_FALSE)){
		bool old_is_accctl = is_acc_control.load();
		is_acc_control.store(req.is_accctl==qcontrol_defs::CommandAction::Request::IS_ACCCTL_TRUE ? true:false);
		if(is_acc_control && !old_is_accctl){
			boost::thread acc_command(&send_acceleration_data);
		}
	}

	if(req.is_pvactl != qcontrol_defs::CommandAction::Request::IS_PVACTL_UNDEFINED &&
			(req.is_pvactl == qcontrol_defs::CommandAction::Request::IS_PVACTL_TRUE || 
					req.is_pvactl == qcontrol_defs::CommandAction::Request::IS_PVACTL_FALSE)){
		bool old_is_pvactl = is_pva_control.load();
		bool new_is_pvactl = req.is_pvactl==qcontrol_defs::CommandAction::Request::IS_PVACTL_TRUE ? true:false;
		if( new_is_pvactl ){
			if(! is_att_control){
				is_att_control.store(true);
				boost::thread attitude_command(&send_attitude_data);
			}
		} else if(old_is_pvactl){
			is_att_control.store(false);
		}
		is_pva_control.store(new_is_pvactl);
		if(is_pva_control && !old_is_pvactl){
			boost::thread pva_command(&send_pva_data);
		}
	}

	if(req.start_takeoff != qcontrol_defs::CommandAction::Request::START_TAKEOFF_UNDEFINED && 
			(req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_TRUE || 
					req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_FALSE)){
		bool old_is_takeoff = is_takeoff;
		bool new_is_takeoff = (req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_TRUE) && is_landed ? true : false;
		if(new_is_takeoff){
			if (! is_vel_control){
				is_vel_control.store(true);
				boost::thread vel_command(&send_velocity_data);
			}
		}else if ( old_is_takeoff){
			is_vel_control.store(false);
		}
		is_takeoff.store(new_is_takeoff);
		if(is_takeoff && !old_is_takeoff ){
			boost::thread take_off_command(&takeoff);
		}
	}

	if(req.start_landing != qcontrol_defs::CommandAction::Request::START_LANDING_UNDEFINED &&
			(req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_TRUE || 
					req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_FALSE)){
		bool old_is_landing = is_landing;
		bool new_is_landing = (req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_TRUE) && !is_landed ? true : false;
		if( new_is_landing ){
			if( ! is_vel_control){
				is_vel_control.store(true);
				boost::thread vel_command(&send_velocity_data);
			}
		}else if(old_is_landing){
			is_vel_control.store(false);
		}
		is_landing.store(new_is_landing);
		if(is_landing && !old_is_landing){
			boost::thread landing_command(&land);
		}
	}

	res.success = true;
	return true;
}

void handle_offboard_info(){
	ros::Rate info_rate(OFFBOARD_INFO_RATE);
	while(ros::ok()){
		qcontrol_defs::QuadState quadState;
		quadState.is_offboard = is_offboard.load();
		quadState.is_armed = is_arm.load();
		quadState.is_posctl = is_pos_control.load();
		quadState.is_attctl = is_pva_control.load() ? false : is_att_control.load();
		quadState.is_velctl = (is_takeoff.load() || is_landing.load()) ? false : is_vel_control.load();
		quadState.is_accctl = is_acc_control.load();
		quadState.is_pvactl = is_pva_control.load();
		quadState.is_takingoff = is_takeoff.load();
		quadState.is_landing = is_landing.load();
		quadState.is_landed = is_landed.load();
		quadState.takeoff_complete = takeoff_complete.load();
		offboard_info.publish(quadState);
		info_rate.sleep();
	}
}

int main(int argc, char **argv){
	//Node initialisation and node handle creation
	ros::init(argc,argv, "offboard_control_node");
	ros::NodeHandle nh = ros::NodeHandle();
	ros::NodeHandle nh_params = ros::NodeHandle("~");

	if(!nh_params.getParam("att_mode_rate",att_mode_rate)){
		att_mode_rate = ATT_MODE_RATE;
		ROS_WARN("No parameter att_mode_rate provided. Using default value %d !",att_mode_rate);
	}

	if(!nh_params.getParam("pva_mode_rate",pva_mode_rate)){
		pva_mode_rate = PVA_MODE_RATE;
		ROS_WARN("No parameter pva_mode_rate provided. Using default value %d !",pva_mode_rate);
	}

	if(!nh_params.getParam("landing_z",landing_z)){
		landing_z = LANDING_Z;
		ROS_WARN("No parameter landing_z provided. Using default value %f !",landing_z);
	}

	if(!nh_params.getParam("takeoff_z",takeoff_z)){
		takeoff_z = TAKEOFF_Z;
		ROS_WARN("No parameter takeoff_z provided. Using default value %f !",takeoff_z);
	}

	if(!nh_params.getParam("takeoff_Kp" , takeoff_Kp)){
		takeoff_Kp = TAKEOFF_KP;
		ROS_WARN("No parameter takeoff_Kp provided. Using default value %f !",takeoff_Kp);
	}

	if(!nh_params.getParam("landing_Kp" , landing_Kp)){
		landing_Kp = LANDING_KP;
		ROS_WARN("No parameter takeoff_Kp provided. Using default value %f !",landing_Kp);
	}

	if(!nh_params.getParam("is_multicopter" , is_mc)){
		is_mc = true;
		ROS_WARN("No parameter is_multicopter provided. Using default value %d !",is_mc);
	}

	//Launching PID params and system params
	readROSparameterServer(pid , pos_param , nh_params);

	//Initialization of subscribers
	ros::Subscriber odom_subscriber = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",10,curr_pos_callback);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state" , 10  , state_callback);
	ros::Subscriber pos_control_sub = nh.subscribe<qcontrol_defs::PosControl>(POS_CONTROL_TOPIC , 10 , pos_callback);
	ros::Subscriber att_control_sub = nh.subscribe<qcontrol_defs::AttControl>(ATT_CONTROL_TOPIC , 10 , att_callback);
	ros::Subscriber vel_control_sub = nh.subscribe<qcontrol_defs::VelControl>(VEL_CONTROL_TOPIC , 10 , vel_callback);
	ros::Subscriber acc_control_sub = nh.subscribe<qcontrol_defs::AccControl>(ACC_CONTROL_TOPIC , 10 , acc_callback);
	ros::Subscriber pva_control_sub = nh.subscribe<qcontrol_defs::PVA>(PVA_CONTROL_TOPIC , 10 , pva_callback);
	ros::Subscriber extended_state = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state" , 10 , land_state_callback);

	//Initialisation of publishers
	target_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel" , 10);
	target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local" ,10);
	// target_att_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude" , 10);
	// target_thrust_pub = nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle" , 10);
	target_att_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/target_attitude" , 10);
	target_thrust_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust" , 10);
	offboard_info = nh.advertise<qcontrol_defs::QuadState>(OFFBOARD_INFO,10);
	if (is_mc)		
		target_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel" , 10);
	else
		target_acc_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control",10);

	//Initialization of services and clients
	ros::ServiceServer command_service = nh.advertiseService(COMMAND_TOPIC, handle_command_action);
	ros::ServiceServer tunning_service = nh.advertiseService(TUNE_CONTROL_TOPIC , updatePosControlParam);
	ros::ServiceServer system_params_service = nh.advertiseService(SYSTEM_PARAMS_TOPIC, updateSystemParam);
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"); 

    //Main loop rate
    ros::Rate main_rate(MAIN_LOOP_RATE);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        main_rate.sleep();
    }

	boost::thread publish_infos(&handle_offboard_info);

    for(int i=0;i<MAIN_LOOP_RATE;i++){
    	ros::spinOnce();
    	main_rate.sleep();
    }

    ROS_WARN("INITIAL YAW : %f", current_yaw);
    ROS_INFO("INITIAL X , Y , Z : %f , %f , %f ",current_odom.pose.pose.position.x , current_odom.pose.pose.position.y , current_odom.pose.pose.position.z);
    is_offboard = true;

    while(ros::ok()){
    	if(is_offboard){
    		set_current_state(OFFBOARD,OFFBOARD_DELAY,false);
    	}
    	ros::spinOnce();
    	main_rate.sleep();
    }

    return 0;
}

