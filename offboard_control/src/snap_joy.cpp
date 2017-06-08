#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/Imu.h>

#include <qcontrol_defs/AttPVA.h>
#include <qcontrol_defs/CommandAction.h>
#include <qcontrol_defs/QuadState.h>

#define COMMAND_TOPIC					"/qcontrol/commands"
#define ATT_POS_TOPIC					"/qcontrol/att_pva"
//#define QUAD_VICON_TOPIC				"/mavros/local_position/pose"
#define QUAD_VICON_TOPIC				"/vicon/"
#define OFFBOARD_INFO					"/qcontrol/quad_info" 

#define MAIN_LOOP_RATE             		70            		//main loop rate for getting more faster all the subscriber datas

#define PI_                   			3.1415					//3.1415926535897    
#define PITCH_MAX               		(PI_/10.0)    		//Max pitch angle allowed
#define ROLL_MAX                		(PI_/10.0)    		//Max roll angle allowed
#define YAW_MAX                    		(PI_/160.0)    		//Max Yaw angle 

#define PITCH_MAX_RATE					0.1
#define ROLL_MAX_RATE					0.1
#define YAW_MAX_RATE 					1.0
#define THROTTLE_MAX             		1.0           		//Max throttle allowed

#define STEP_Z_RATE                     0.05
#define STEP_Y_X_RATE                   0.05

//Joystick configuration

#define THROTTLE_AXE            		1            		//Up/Down left axis
#define YAW_LEFT_AXE            		2            		//LT
#define YAW_RIGHT_AXE            		5            		//RT
#define PITCH_AXE                		4            		//Up/down right axis          
#define ROLL_AXE                		3            		//left/right right axis

#define SPEED_TO_POS_CONTROL			5					//RB

#define ARM_MOTOR                		7            		//Start button
#define AUTOHLD_OFFB					6					//back button
#define POSCTL 							2					//X button
#define ATTCTL 							0					//A button
#define TAKEOFF							1					//B button
#define LAND							3					//Y button

#define STEP_Z_INCR                		13            		//cross key up
#define STEP_Z_DECR                		14            		//cross key down
#define STEP_LEFT_RIGHT_INCR    		12            		//cross key right
#define STEP_LEFT_RIGHT_DECR    		11            		//cross key left 	   



ros::Publisher att_pos_pub;
ros::ServiceClient command_client;

//Position control step for  x,y and z axis
double max_speed_z = 1.8;
double max_speed_y = 1.6;
double max_speed_x = 1.6;

double max_z = 0.05;
double max_y = 0.05;
double max_x = 0.03;

//Main values to send like position, velocity and accelearation
qcontrol_defs::AttPVA current_data;
qcontrol_defs::CommandAction command_msgs;
qcontrol_defs::QuadState quad_state;

geometry_msgs::Point mc_given_pos;
geometry_msgs::Point current_pos;

double mc_given_yaw;
double current_yaw;

//save last buttons state
bool last_is_arm = false;
bool last_offboard = false;
bool last_attitude = false;
bool last_posctl  = false;
bool last_takeoff = false;
bool last_land = false;
bool last_speed_to_pos = false;

bool use_speed = true;


//Main joy callback function for dealing with Joystick event
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){

	command_msgs = qcontrol_defs::CommandAction();
    
    bool is_offboard = msg->buttons[AUTOHLD_OFFB] == 1 ? true : false;
    bool is_arm = msg->buttons[ARM_MOTOR] == 1 ?true : false;
    bool is_attitude = msg->buttons[ATTCTL] == 1 ? true : false;
    bool is_posctl = msg->buttons[POSCTL] == 1 ? true : false;
    bool is_takeoff = msg->buttons[TAKEOFF] == 1 ? true : false;
    bool is_land = msg->buttons[LAND] == 1 ? true : false;

    bool is_speed_to_pos = msg->buttons[SPEED_TO_POS_CONTROL] == 1 ? true : false;

    bool send_command = false;

    if(is_offboard && !last_offboard){
    	command_msgs.request.is_poshld= (quad_state.is_offboard)? 1 : 2;
    	send_command = true;
    }

    if(is_arm && !last_is_arm){
    	command_msgs.request.arm_motors = (!quad_state.is_armed) ? 1 : 2;
    	send_command = true;
    }

    if(is_attitude && !last_attitude){
    	command_msgs.request.is_attctl = (!quad_state.is_attctl) ? 1 : 2; 
    	//current_yaw = mc_given_yaw;
    	send_command = true;
    }

    if(is_posctl && !last_posctl){
    	command_msgs.request.is_posctl = (!quad_state.is_posctl) ? 1 : 2;
    	//current_yaw = mc_given_yaw;
    	send_command = true;
    }

    if(is_takeoff && !last_takeoff){
    	command_msgs.request.start_takeoff = (!quad_state.is_takingoff) ? 1 : 2;
    	send_command = true;
    }

    if(is_land && !last_land){
    	command_msgs.request.start_landing = (!quad_state.is_landing) ? 1 : 2;
    	send_command = true;
    }
    
    if(is_speed_to_pos && !last_speed_to_pos){
    	use_speed = !use_speed;
    	current_pos = mc_given_pos;
    }

    if(quad_state.is_attctl){
    	current_data.use_position = false;
    	current_data.use_rate = false;
    	if(current_data.use_rate){
    		current_data.posZ_thrust = ((msg->axes[THROTTLE_AXE]>=0.0 ? msg->axes[THROTTLE_AXE]:0.0))*THROTTLE_MAX;
    		current_data.posX_roll =  -msg->axes[ROLL_AXE]	*	ROLL_MAX_RATE; //a modifier
    		current_data.posY_pitch = -msg->axes[PITCH_AXE]	*	PITCH_MAX_RATE;
    		current_data.yaw = ((-(msg->axes[YAW_LEFT_AXE]-1.0)/2.0) + ((msg->axes[YAW_RIGHT_AXE]-1.0)/2.0))	*	YAW_MAX_RATE;
            current_data.use_yaw=false;
    	}else {
    		current_data.posZ_thrust = ((msg->axes[THROTTLE_AXE]>=0.0 ? msg->axes[THROTTLE_AXE]:0.0))*THROTTLE_MAX;
    		current_data.posX_roll =  msg->axes[ROLL_AXE]	*	ROLL_MAX; //a modifier
    		current_data.posY_pitch = msg->axes[PITCH_AXE]	*	PITCH_MAX;
    		current_yaw += ((-(msg->axes[YAW_LEFT_AXE]-1.0)/2.0) + ((msg->axes[YAW_RIGHT_AXE]-1.0)/2.0))	*	YAW_MAX;
    		current_data.yaw = current_yaw;
            current_data.use_yaw=false;
    	}

    } else if(quad_state.is_posctl){
    	//current_data.use_position = false;
    	current_data.use_rate = true;

    	//current_yaw += (-msg->axes[YAW_LEFT_AXE] + msg->axes[YAW_RIGHT_AXE])	*	YAW_MAX;
    	//current_yaw = current_yaw > PI_ ?(-2*PI_ +current_yaw) : (current_yaw <-PI_ ? (2*PI_ + current_yaw) : current_yaw);

    	if(current_data.use_rate){
    		current_data.use_yaw = false;
    		current_data.yaw = ((-(msg->axes[YAW_LEFT_AXE]-1.0)/2.0) + ((msg->axes[YAW_RIGHT_AXE]-1.0)/2.0))	*	YAW_MAX_RATE;
    	}else {
    		current_data.use_yaw = true;
    		current_yaw += ((-(msg->axes[YAW_LEFT_AXE]-1.0)/2.0) + ((msg->axes[YAW_RIGHT_AXE]-1.0)/2.0))	*	YAW_MAX;
    		current_data.yaw = current_yaw;
    	}

    	if(use_speed){
    		current_data.use_speed = true;
    		current_data.use_position = false;
    		current_data.use_acceleration = false;
    		current_data.use_body_frame_offset = true;
	    	current_data.velX = msg->axes[PITCH_AXE] 	*	max_speed_x;
    		current_data.velY = msg->axes[ROLL_AXE]	*	max_speed_y;
    		current_data.velZ = msg->axes[THROTTLE_AXE]	*	max_speed_z;
    		max_speed_z += msg->buttons[STEP_Z_INCR]*STEP_Z_RATE - msg->buttons[STEP_Z_DECR]*STEP_Z_RATE;
    		max_speed_x += msg->buttons[STEP_LEFT_RIGHT_INCR]*STEP_Y_X_RATE - msg->buttons[STEP_LEFT_RIGHT_DECR]*STEP_Y_X_RATE;
    		max_speed_y += msg->buttons[STEP_LEFT_RIGHT_INCR]*STEP_Y_X_RATE - msg->buttons[STEP_LEFT_RIGHT_DECR]*STEP_Y_X_RATE;
    	} else {
    		current_data.use_speed = false;
    		current_data.use_position = true;
    		current_data.use_acceleration = false;
    		current_data.use_body_frame_offset = false;
            if(current_data.use_body_frame_offset){
                current_data.posX_roll = msg->axes[PITCH_AXE] * max_x;
                current_data.posY_pitch = msg->axes[ROLL_AXE] * max_y;
                current_data.posZ_thrust =  msg->axes[THROTTLE_AXE] * max_z;
            }else {
                current_pos.x += msg->axes[PITCH_AXE] * max_x;
                current_pos.y += msg->axes[ROLL_AXE] * max_y;
                current_pos.z += msg->axes[THROTTLE_AXE] * max_z;
                current_data.posZ_thrust = current_pos.z;
                current_data.posY_pitch = current_pos.y;
                current_data.posX_roll = current_pos.x;
            }


            max_z += msg->buttons[STEP_Z_INCR]*STEP_Z_RATE - msg->buttons[STEP_Z_DECR]*STEP_Z_RATE;
            max_x += msg->buttons[STEP_LEFT_RIGHT_INCR]*STEP_Y_X_RATE - msg->buttons[STEP_LEFT_RIGHT_DECR]*STEP_Y_X_RATE;
            max_y += msg->buttons[STEP_LEFT_RIGHT_INCR]*STEP_Y_X_RATE - msg->buttons[STEP_LEFT_RIGHT_DECR]*STEP_Y_X_RATE;
    	}

    }

    if(send_command){
    	command_client.call(command_msgs);
    }

	if ( msg->buttons[STEP_Z_INCR]==1 || msg->buttons[STEP_Z_DECR]==1 || msg->buttons[STEP_LEFT_RIGHT_INCR]==1
    								|| msg->buttons[STEP_LEFT_RIGHT_DECR]==1){
		if(use_speed){
    		ROS_WARN("[POSITION CONTROL] CURRENT Z , Y , X SPEED RATES : %f   %f   %f",max_speed_z , max_speed_y , max_speed_x);
    	}else {
    		ROS_WARN("[POSITION CONTROL] CURRENT Z , Y , X POS RATES : %f   %f   %f",max_z , max_y , max_x);
    	}
    }

    last_is_arm = is_arm;
    last_offboard = is_offboard;
    last_attitude = is_attitude;
    last_posctl = is_posctl;
    last_takeoff = is_takeoff;
    last_land = is_land;
    last_speed_to_pos = is_speed_to_pos;

    //Publish the data here since only one thread !!!
    att_pos_pub.publish(current_data);
}

static void toEulerianAngle(const geometry_msgs::Quaternion& q, double& yaw)
{
    double ysqr = q.y * q.y;

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
    yaw = std::atan2(t3, t4);
}

/*void curr_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//Must change this for vicon system --> TransformStamped
	mc_given_pos = msg->pose.position;
	toEulerianAngle(msg->pose.orientation,mc_given_yaw);
}*/

void curr_pos_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	//Must change this for vicon system --> TransformStamped
	mc_given_pos.x = msg->transform.translation.x;
	mc_given_pos.y = msg->transform.translation.y;
	mc_given_pos.z = msg->transform.translation.z;
	toEulerianAngle(msg->transform.rotation,mc_given_yaw);
}

void current_state_callback(const qcontrol_defs::QuadState::ConstPtr& msg){
	quad_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"joy_command_node");
    ros::NodeHandle nh("~");

    std::string quad_name;
    if(!nh.getParam("quad_name",quad_name)){
        quad_name = "Quad8";
        ROS_WARN("No quad name provided. Default quad name is %s",quad_name.c_str());
    }

    //Subscribers
    ros::Subscriber joy_sub =nh.subscribe<sensor_msgs::Joy>(("/"+quad_name+"/joy").c_str(),10,joy_callback);
    //ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(QUAD_VICON_TOPIC,10,curr_pos_callback); // use the vicon for this
    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::TransformStamped>((QUAD_VICON_TOPIC+quad_name+"/"+quad_name).c_str(),10,curr_pos_callback);
    ros::Subscriber current_state = nh.subscribe<qcontrol_defs::QuadState>(("/"+quad_name+OFFBOARD_INFO).c_str(),10,current_state_callback);

    //Publishers
    att_pos_pub = nh.advertise<qcontrol_defs::AttPVA>(("/"+quad_name+ATT_POS_TOPIC).c_str(),10);

    //Services and clients
    command_client= nh.serviceClient<qcontrol_defs::CommandAction>(("/"+quad_name+COMMAND_TOPIC).c_str());

    //Main loop rate
    ros::Rate main_rate(MAIN_LOOP_RATE);

    for (int i=0; i<200;i++){
    	ros::spinOnce();
    	main_rate.sleep();
    }

    //setting initial yaw and initial position
	current_yaw = mc_given_yaw;
	current_pos = mc_given_pos;

	ROS_WARN("[JOY NODE ]:  initial yaw : %f",current_yaw);
    while(ros::ok()){
    	ros::spinOnce();
    	main_rate.sleep();
    }

    return 0;
}