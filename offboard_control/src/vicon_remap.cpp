
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#define SENDING_DATA_RATE 70 //in Hertz

ros::Publisher current_pos_pub; //Publisher of the position
geometry_msgs::PoseStamped current_pos;

long target_seq = 0;

void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){

	//Position and orientation values
	/*current_pos.pose.position.x = msg->transform.translation.x;
	current_pos.pose.position.y = msg->transform.translation.y;
	current_pos.pose.position.z = msg->transform.translation.z;
	current_pos.pose.orientation = msg->transform.rotation;

	//Setting header with curent ros time
	current_pos.header.seq = target_seq;
	current_pos.header.frame_id = "fcu";
	current_pos.header.stamp = ros::Time::now();*/

    current_pos.pose.position.x = msg->transform.translation.x;
    current_pos.pose.position.y = msg->transform.translation.y;
    current_pos.pose.position.z = msg->transform.translation.z;
    current_pos.pose.orientation = msg->transform.rotation;

    //Setting header with curent ros time
    current_pos.header.seq = msg->header.seq;
    current_pos.header.frame_id = msg->header.frame_id;
    current_pos.header.stamp = msg->header.stamp;

	current_pos_pub.publish(current_pos);
    //current_pos_pub.publish(*msg);
	target_seq++;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "vicon_remap_node");
    ros::NodeHandle nh_params("~"); //Node handle for this node
    ros::NodeHandle nh = ros::NodeHandle();

    std::string quad_name ;

    if(!nh_params.getParam("quad_name",quad_name)){
    	quad_name = "Quad8";
    	ROS_WARN("No quad name provided. Default quad name is %s",quad_name.c_str());
    }

    int remap_freq;
    if(!nh_params.getParam("frequency",remap_freq)){
    	remap_freq = SENDING_DATA_RATE;
    	ROS_WARN("No remap frequency provided. Default value is %d",remap_freq);
    }

	//Subscribe to the appropriate vicon topics for position and orientation
    ros::Subscriber vicon_sub =nh.subscribe<geometry_msgs::TransformStamped>(("/vicon/"+quad_name+"/"+quad_name).c_str(),10,vicon_callback);

	//Sending data to the mocap/pose for mocap
    current_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(("/"+quad_name+"/mavros/mocap/pose").c_str(), 10);
    //current_pos_pub = nh.advertise<geometry_msgs::TransformStamped>("mavros/mocap/tf", 10 );

    ros::Rate rate(remap_freq); // SENDING_DATA_RATE Hz as the frequency of sending data to PX4

    ROS_WARN("SENDING VICON of %s POSITION TO PX4 STARTED !!!!!",quad_name.c_str());

    while (ros::ok()){

    	ros::spinOnce();

    	rate.sleep();
    }
    ROS_WARN("SENDING VICON of %s POSITION NODE STOPPED !!!!!",quad_name.c_str());

}