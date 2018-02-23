#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
//#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Quaternion.h>

#include <qcontrol_defs/AttPVA.h>
#include <qcontrol_defs/CommandAction.h>
#include <qcontrol_defs/QuadState.h>
//#include <sensor_msgs/Imu.h>

#include <std_msgs/Float64.h>

#include <atomic>
#include <boost/thread.hpp>
#include <iostream>
#include <string>

#define COMMAND_TOPIC					"/qcontrol/commands"
#define ATT_POS_TOPIC					"/qcontrol/att_pva"
#define OFFBOARD_INFO					"/qcontrol/quad_info" 

#define MAIN_LOOP_RATE             		100            //main loop rate for getting more faster all the subscriber datas
#define ATT_MODE_RATE					10
#define POS_MODE_RATE					10
#define OFFBOARD_DELAY					0.5
#define OFFBOARD_INFO_RATE 				10

#define OFFBOARD                 		"OFFBOARD"
#define AUTO_LOITER                    	"AUTO.LOITER"

#define TAKEOFF_RATE 					10
#define LANDING_RATE					10
#define TAKEOFF_Z						0.8
#define TAKEOFF_SPEED					1.2
#define TAKEOFF_DETECTED				0.3
#define LANDING_Z						0.12
#define PI_ 							3.1415

//Some publisher call global variables
ros::Publisher target_raw_att_pub;
ros::Publisher target_raw_pos_pub;
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
std::atomic<bool> is_attitude_control(false);
std::atomic<bool> is_PosCtrl_control(false);
std::atomic<bool> is_arm (false);
std::atomic<bool> is_takeoff(false);
std::atomic<bool> is_landing(false);
std::atomic<bool> is_landed(true);
std::atomic<bool> takeoff_complete(false);


std::atomic<bool> is_attitude_control_thread(false);
std::atomic<bool> is_PosCtrl_control_thread(false);
std::atomic<bool> is_takeoff_thread(false);
std::atomic<bool> is_landing_thread(false);

//mutex for locking both access to data commands sent
boost::mutex mutex_cmd_;
boost::mutex mutex_curr_pos_;

//Main values to send like position, velocity and accelearation
qcontrol_defs::AttPVA current_data;

//Attitude message to publish
mavros_msgs::AttitudeTarget att_msg;

//PVA message to publish
mavros_msgs::PositionTarget pos_msg;

//current yaw value and position given by the fcu
double current_yaw;
double initial_yaw;
geometry_msgs::Point current_pos;

//Long variable for sequence count in header
long att_seq = 0;
long pos_seq = 0;

//last request time for services call
ros::Time last_request;

//Parameters variables
double landing_z;
double takeoff_z;
double takeoff_speed;
int att_mode_rate;
int pos_mode_rate;


static void fromPVAToLocalNED(geometry_msgs::Vector3 &res, const double posZ,const double posY,const double posX){
	res.x = posX;
	res.y = posY;
	res.z = posZ;
}

static void fromOrientationtoLocalNED(geometry_msgs::Vector3 &res, const double pitch,const double roll,const double yaw){
	res.x = -roll;
	res.y = pitch;
	res.z = yaw;
}

static void fromPVAToBodyLocalNED(geometry_msgs::Vector3 &res, const double posZ,const double posY,const double posX){
	res.x = -posY;
	res.y = posX;
	res.z = posZ;
}

//Eulerian angle to Quaternions --> Here the coordinate system is X forward, Y right and Z down
//roll = X axis , pitch = Y axis and Yaw = Z axis
static void toQuaternion(geometry_msgs::Quaternion& q, double pitch, double roll, double yaw)
{
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
}

static void toEulerianAngle(const geometry_msgs::Quaternion& q, double& yaw)
{
    double ysqr = q.y * q.y;

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    yaw = std::atan2(t3, t4);
}

void stay_current_pos(){
	current_data.use_position = true;
	current_data.use_speed = false;
	current_data.use_acceleration = false;
	current_data.use_yaw = false;
	current_data.use_rate = true;
	current_data.posZ_thrust = current_pos.z;
	current_data.posY_pitch = current_pos.y;
	current_data.posX_roll = current_pos.x;
	current_data.yaw = initial_yaw;
}

void wait_thread_to_stop(std::atomic<bool> &associate_var,std::atomic<bool> &curr_thread){
	associate_var.store(false);
	while (curr_thread);
}

//Typically set the mode between AUTO_LOITER and OFFBOARD MODE
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

//send and arm request and doesn't stop the function until it have finished
void set_arming(const bool is_arm_){
    arm_cmd.request.value = is_arm_;
    ros::Rate m_rate(20);
    while (ros::ok() && !(arming_client.call(arm_cmd) && arm_cmd.response.mode_sent)){
        m_rate.sleep();
    }
    if(!is_arm_){
    	is_takeoff.store(false);
    	is_landing.store(false);
    	is_PosCtrl_control.store(false);
    	is_attitude_control.store(false);
    }
    ROS_WARN("MOTORS ARE NOW %s !!!!!",is_arm_?"ARMED":"DISARMED");
}

void send_attitude_data(){
	is_offboard.store(true);
	is_attitude_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_PosCtrl_control,is_PosCtrl_control_thread);

	ROS_WARN("ATTITUDE CONTROL MODE STARTED !!!!!");
	ros::Rate att_rate(att_mode_rate);
	while(ros::ok() && is_offboard && is_attitude_control){

		geometry_msgs::Vector3 converted_orientation;
		double pitch , roll , yaw , throttle;

		mutex_cmd_.lock();
		pitch = current_data.posY_pitch;
		yaw = current_data.yaw;
		roll = current_data.posX_roll;
		throttle = current_data.posZ_thrust;

		bool use_position = current_data.use_position;
		bool use_rate = current_data.use_rate;
		mutex_cmd_.unlock();

		if(use_rate){
			att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
			att_msg.body_rate.x = roll;
			att_msg.body_rate.y = pitch;
			att_msg.body_rate.z = yaw;
		}else {
			att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
								mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
								mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
			fromOrientationtoLocalNED(converted_orientation,pitch,roll,yaw);
			toQuaternion(att_msg.orientation,converted_orientation.y,converted_orientation.x,converted_orientation.z);
		}

		att_msg.thrust = throttle;
		att_msg.header.seq = att_seq;
		att_msg.header.stamp = ros::Time::now();
		att_msg.header.frame_id="1";
		if(!use_position){
			target_raw_att_pub.publish(att_msg);
			att_seq++;
		}
		att_rate.sleep();
	}

	ROS_INFO("ATTITUDE CONTROL MODE STOPPED !!!!!!");
	is_attitude_control.store(false);
	is_attitude_control_thread.store(false);
}

void send_target_position(){
	is_offboard.store(true);
	is_PosCtrl_control_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_attitude_control,is_attitude_control_thread);
	wait_thread_to_stop(is_landing,is_landing_thread);

	ROS_WARN("POSITION CONTROL MODE STARTED !!!!!");
	ros::Rate pos_rate(pos_mode_rate);
	pos_msg = mavros_msgs::PositionTarget();
	mutex_cmd_.lock();
	initial_yaw = current_yaw;
	stay_current_pos();
	mutex_cmd_.unlock();

	geometry_msgs::Vector3 pos, speed , acc;
	while(ros::ok() && is_offboard && is_PosCtrl_control){

		mutex_cmd_.lock();
		pos_msg.yaw =  current_data.yaw - (PI_/2); //Always invert yaw !!!!
		pos_msg.yaw_rate = current_data.yaw;
		bool use_position = current_data.use_position;
		bool use_acceleration = current_data.use_acceleration;
		bool use_speed = current_data.use_speed;	
		bool use_yaw = current_data.use_yaw;
		bool use_rate = current_data.use_rate;

		if(current_data.use_body_frame_offset){
			pos_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
			fromPVAToBodyLocalNED(pos,current_data.posZ_thrust,current_data.posY_pitch,current_data.posX_roll);
			fromPVAToBodyLocalNED(acc,current_data.accZ,current_data.accY,current_data.accX);
			fromPVAToBodyLocalNED(speed,current_data.velZ,current_data.velY,current_data.velX);
		}else {
			pos_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			fromPVAToLocalNED(pos,current_data.posZ_thrust,current_data.posY_pitch,current_data.posX_roll);
			fromPVAToLocalNED(acc,current_data.accZ,current_data.accY,current_data.accX);
			fromPVAToLocalNED(speed,current_data.velZ,current_data.velY,current_data.velX);
		}
		mutex_cmd_.unlock();

		pos_msg.position.x = pos.x;
		pos_msg.position.y = pos.y;
		pos_msg.position.z = pos.z;
		pos_msg.velocity = speed;
		pos_msg.acceleration_or_force = acc;

		pos_msg.type_mask = 0;
		if(!use_rate){
			pos_msg.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
			pos_msg.yaw_rate = 0;
		}
		if(!use_position){
			pos_msg.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX |
								mavros_msgs::PositionTarget::IGNORE_PY |
								mavros_msgs::PositionTarget::IGNORE_PZ ;
		}
		if(!use_speed){
			pos_msg.type_mask |= mavros_msgs::PositionTarget::IGNORE_VX |
								mavros_msgs::PositionTarget::IGNORE_VY |
								mavros_msgs::PositionTarget::IGNORE_VZ ;
		}
		if(!use_acceleration){
			pos_msg.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX |
								mavros_msgs::PositionTarget::IGNORE_AFY |
								mavros_msgs::PositionTarget::IGNORE_AFZ ;
		}
		if(!use_yaw){
			pos_msg.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
			pos_msg.yaw =  0; //Always invert yaw !!!!
		}

		pos_msg.header.seq = pos_seq;
		pos_msg.header.stamp= ros::Time::now();
		pos_msg.header.frame_id = "1";

		target_raw_pos_pub.publish(pos_msg);
		pos_seq++;
		pos_rate.sleep();
	}

	ROS_INFO("POSITION CONTROL MODE STOPPED !!!!!");
	is_PosCtrl_control.store(false);
	is_PosCtrl_control_thread.store(false);
}

void takeoff(){
	is_offboard.store(true);
	is_takeoff_thread.store(true);
	takeoff_complete.store(false);

	wait_thread_to_stop(is_landing,is_landing_thread);
	wait_thread_to_stop(is_attitude_control,is_attitude_control_thread);
	wait_thread_to_stop(is_PosCtrl_control,is_PosCtrl_control_thread);

	ROS_WARN("STARTING TAKING OFF THREAD......");
	pos_msg = mavros_msgs::PositionTarget();

	mutex_curr_pos_.lock();
	pos_msg.position.x = current_pos.x;
	pos_msg.position.y = current_pos.y;
	initial_yaw = current_yaw;
	mutex_curr_pos_.unlock();

	pos_msg.position.z = takeoff_z;
	pos_msg.yaw = initial_yaw - PI_/2;
	pos_msg.yaw_rate = 0.0;
	//pos_msg.velocity.x = 0.0;
	//pos_msg.velocity.y = 0.0;
	//pos_msg.velocity.z = takeoff_speed;
	//pos_msg.velocity.z = 0.0;

	pos_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	//pos_msg.type_mask  = 	0x1000 | mavros_msgs::PositionTarget::IGNORE_AFZ
	//							   | mavros_msgs::PositionTarget::IGNORE_PZ;
	pos_msg.type_mask  = 	mavros_msgs::PositionTarget::IGNORE_AFZ
								| mavros_msgs::PositionTarget::IGNORE_AFY
								| mavros_msgs::PositionTarget::IGNORE_AFX
								| mavros_msgs::PositionTarget::IGNORE_VX
								| mavros_msgs::PositionTarget::IGNORE_VY
								| mavros_msgs::PositionTarget::IGNORE_YAW_RATE
								| mavros_msgs::PositionTarget::IGNORE_VZ;
	set_arming(true); //arm the motors before taking off

	ros::Rate takeoff_rate(TAKEOFF_RATE);
	bool takeoff_reach = false;

	while(ros::ok() && is_takeoff && is_offboard){
		mutex_curr_pos_.lock();
		if(current_pos.z >= TAKEOFF_DETECTED && !takeoff_reach){
			takeoff_reach = true;
			takeoff_complete.store(true);
			ROS_WARN("TAKING OFF COMPLETE !!!!!");
			/*pos_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			pos_msg.type_mask  = 	mavros_msgs::PositionTarget::IGNORE_AFZ
								| mavros_msgs::PositionTarget::IGNORE_AFY
								| mavros_msgs::PositionTarget::IGNORE_AFX
								| mavros_msgs::PositionTarget::IGNORE_VX
								| mavros_msgs::PositionTarget::IGNORE_VY
								| mavros_msgs::PositionTarget::IGNORE_VZ;*/
		}
		mutex_curr_pos_.unlock();
		pos_msg.header.seq = pos_seq;
		pos_msg.header.stamp= ros::Time::now();
		pos_msg.header.frame_id = "1";
		target_raw_pos_pub.publish(pos_msg);
		pos_seq++;
		takeoff_rate.sleep();
	}
	is_takeoff.store(false);
	is_takeoff_thread.store(false);
	takeoff_complete.store(false);
	ROS_WARN("STOPPING TAKING OFF THREAD !!!!!");
}

void land(){
	is_offboard.store(true);
	is_landing_thread.store(true);
	wait_thread_to_stop(is_takeoff,is_takeoff_thread);
	wait_thread_to_stop(is_attitude_control,is_attitude_control_thread);
	wait_thread_to_stop(is_PosCtrl_control,is_PosCtrl_control_thread);

	ROS_WARN("STARTING LANDING MODE THREAD......");

	pos_msg = mavros_msgs::PositionTarget();

	mutex_curr_pos_.lock();
	pos_msg.position.x = current_pos.x;
	pos_msg.position.y = current_pos.y;
	mutex_curr_pos_.unlock();
	pos_msg.position.z = 0.0;
	pos_msg.yaw_rate = 0.0;
	pos_msg.type_mask = 0x2000   | mavros_msgs::PositionTarget::IGNORE_YAW
                                 | mavros_msgs::PositionTarget::IGNORE_AFZ;
	ros::Rate landing_rate(LANDING_RATE);
	while(ros::ok() && is_landing && is_offboard){
		mutex_curr_pos_.lock();
		if(current_pos.z < landing_z){
			mutex_curr_pos_.unlock();
			set_arming(false);
			break;
		}
		mutex_curr_pos_.unlock();
		pos_msg.header.seq = pos_seq;
		pos_msg.header.stamp= ros::Time::now();
		pos_msg.header.frame_id = "1";
		target_raw_pos_pub.publish(pos_msg);
		pos_seq++;
		landing_rate.sleep();
	}
	is_landing.store(false);
	is_landing_thread.store(false);
	ROS_WARN("STOPPING LANDING MODE THREAD !!!!!!");
}

//State callback function 
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state= *msg;
    is_arm.store(current_state.armed);
}

//Attitude, position , commands callback function
void pos_att_callback(const qcontrol_defs::AttPVA::ConstPtr& msg){
	mutex_cmd_.lock();
	current_data = *msg;
	mutex_cmd_.unlock();
}

//void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
//	toEulerianAngle(msg->orientation,current_yaw);
//}

void curr_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	mutex_curr_pos_.lock();
	current_pos = msg->pose.position;
	toEulerianAngle(msg->pose.orientation,current_yaw);
	mutex_curr_pos_.unlock();
}

bool handle_command_action(qcontrol_defs::CommandAction::Request &req, qcontrol_defs::CommandAction::Response &res){

	if(req.is_poshld != qcontrol_defs::CommandAction::Request::IS_POSHLD_UNDEFINED &&
			(req.is_poshld == qcontrol_defs::CommandAction::Request::IS_POSHLD_TRUE || 
					req.is_poshld == qcontrol_defs::CommandAction::Request::IS_POSHLD_FALSE)){
		is_offboard.store(req.is_poshld==qcontrol_defs::CommandAction::Request::IS_POSHLD_TRUE?false:true);
		set_current_state(!is_offboard?AUTO_LOITER:OFFBOARD,0.1,false);
	}

	if(req.arm_motors != qcontrol_defs::CommandAction::Request::ARM_MOTOR_UNDEFINED &&
			(req.arm_motors == qcontrol_defs::CommandAction::Request::ARM_MOTOR_TRUE || 
					req.arm_motors == qcontrol_defs::CommandAction::Request::ARM_MOTOR_FALSE)){
		set_arming(req.arm_motors==qcontrol_defs::CommandAction::Request::ARM_MOTOR_TRUE?true:false);
	}

	if(req.is_attctl != qcontrol_defs::CommandAction::Request::IS_ATTCTL_UNDEFINED &&
			(req.is_attctl == qcontrol_defs::CommandAction::Request::IS_ATTCTL_TRUE || 
					req.is_attctl == qcontrol_defs::CommandAction::Request::IS_ATTCTL_FALSE)){
		bool old_is_attitude = is_attitude_control.load();
		is_attitude_control.store(req.is_attctl==qcontrol_defs::CommandAction::Request::IS_ATTCTL_TRUE?true:false);
		if(is_attitude_control && !old_is_attitude){
			boost::thread attitude_command(&send_attitude_data);
		}
	}
	
	if(req.is_posctl != qcontrol_defs::CommandAction::Request::IS_POSCTL_UNDEFINED &&
			(req.is_posctl == qcontrol_defs::CommandAction::Request::IS_POSCTL_TRUE || 
					req.is_posctl == qcontrol_defs::CommandAction::Request::IS_POSCTL_FALSE)){
		bool old_is_posctl = is_PosCtrl_control.load();
		is_PosCtrl_control.store(req.is_posctl==qcontrol_defs::CommandAction::Request::IS_POSCTL_TRUE ? true:false);
		if(is_PosCtrl_control && !old_is_posctl){
			boost::thread pos_command(&send_target_position);
		}
	}

	if(req.start_takeoff != qcontrol_defs::CommandAction::Request::START_TAKEOFF_UNDEFINED && 
			(req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_TRUE || 
					req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_FALSE)){
		bool old_is_takeoff = is_takeoff;
		is_takeoff.store((req.start_takeoff == qcontrol_defs::CommandAction::Request::START_TAKEOFF_TRUE) && is_landed ? true : false);
		if(is_takeoff && !old_is_takeoff ){
			boost::thread take_off_command(&takeoff);
		}
	}

	if(req.start_landing != qcontrol_defs::CommandAction::Request::START_LANDING_UNDEFINED &&
			(req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_TRUE || 
					req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_FALSE)){
		bool old_is_landing = is_landing;
		is_landing.store((req.start_landing == qcontrol_defs::CommandAction::Request::START_LANDING_TRUE) && !is_landed ? true : false);
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
		quadState.is_posctl = is_PosCtrl_control.load();
		quadState.is_attctl = is_attitude_control.load();
		quadState.is_takingoff = is_takeoff.load();
		quadState.is_landing = is_landing.load();
		quadState.is_landed = is_landed.load();
		quadState.takeoff_complete = takeoff_complete.load();
		offboard_info.publish(quadState);
		info_rate.sleep();
	}
}

void land_state_callback(const mavros_msgs::ExtendedState::ConstPtr& msg){
	if(msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
		is_landed.store(true);
	}else if(msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR){
		is_landed.store(false);
	}
}

int main(int argc, char **argv){
	//Node initialisation and node handle creation
	ros::init(argc,argv, "offboard_control_node");
	ros::NodeHandle nh("~");

	if(!nh.getParam("att_mode_rate",att_mode_rate)){
		att_mode_rate = ATT_MODE_RATE;
		ROS_WARN("No parameter att_mode_rate provided. Using default value %d !",att_mode_rate);
	}

	if(!nh.getParam("pos_mode_rate",pos_mode_rate)){
		pos_mode_rate = POS_MODE_RATE;
		ROS_WARN("No parameter pos_mode_rate provided. Using default value %d !",pos_mode_rate);
	}

	if(!nh.getParam("landing_z",landing_z)){
		landing_z = LANDING_Z;
		ROS_WARN("No parameter landing_z provided. Using default value %f !",landing_z);
	}

	if(!nh.getParam("takeoff_z",takeoff_z)){
		takeoff_z = TAKEOFF_Z;
		ROS_WARN("No parameter takeoff_z provided. Using default value %f !",takeoff_z);
	}

	if(!nh.getParam("takeoff_speed",takeoff_speed)){
		takeoff_speed = TAKEOFF_SPEED;
		ROS_WARN("No parameter takeoff_speed provided. Using default value %f !",takeoff_speed);
	}

	std::string quad_name;
	if(!nh.getParam("quad_name",quad_name)){
		quad_name = "Quad8";
		ROS_WARN("No quad name provided. Default quad name is %s",quad_name.c_str());
	}

	//Subscribers
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(("/"+quad_name+"/mavros/state").c_str(),1,state_callback);
	ros::Subscriber pos_att_sub = nh.subscribe<qcontrol_defs::AttPVA>(("/"+quad_name+ATT_POS_TOPIC).c_str(),1,pos_att_callback);
	ros::Subscriber pos_current_sub = nh.subscribe<geometry_msgs::PoseStamped>(("/"+quad_name+"/mavros/local_position/pose").c_str(),1,curr_pos_callback);
	ros::Subscriber extended_state = nh.subscribe<mavros_msgs::ExtendedState>(("/"+quad_name+"/mavros/extended_state").c_str(),1,land_state_callback);
	//ros::Subscriber orientation_current_sub = nh.subscribe<sensor_msgs::Imu>("/mavro/imu/data",10,imu_callback);

	//Publishers
    target_raw_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(("/"+quad_name+"/mavros/setpoint_raw/local").c_str(),1);
    target_raw_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>(("/"+quad_name+"/mavros/setpoint_raw/attitude").c_str(),1);
    offboard_info = nh.advertise<qcontrol_defs::QuadState>(("/"+quad_name+OFFBOARD_INFO).c_str(),1);

    //Services and clients
    ros::ServiceServer command_service = nh.advertiseService(("/"+quad_name+COMMAND_TOPIC).c_str(), handle_command_action);
    arming_client= nh.serviceClient<mavros_msgs::CommandBool>(("/"+quad_name+"/mavros/cmd/arming").c_str());
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(("/"+quad_name+"/mavros/set_mode").c_str());

    //Main loop rate
    ros::Rate main_rate(MAIN_LOOP_RATE);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        main_rate.sleep();
    }
	boost::thread publish_infos(&handle_offboard_info);

    for(int i=0;i<200;i++){
    	ros::spinOnce();
    	main_rate.sleep();
    }
    initial_yaw = current_yaw;
    ROS_WARN("INITIAL YAW : %f",initial_yaw);
    //Starting with AUTO_LOITER for security if we lose signal or get off of offboard mode
    set_current_state(AUTO_LOITER,0.1,true);
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

