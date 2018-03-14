
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#define SENDING_DATA_RATE 100 //in Hertz

ros::Publisher current_pos_pub; //Publisher of the position
geometry_msgs::TransformStamped current_pos;

long target_seq = 0;

void vicon_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	//Position and orientation values
    current_pos.transform.translation.x = msg->pose.position.x;
    current_pos.transform.translation.y = msg->pose.position.y;
    current_pos.transform.translation.z = msg->pose.position.z;
    current_pos.transform.rotation = msg->pose.orientation;

	//Setting header with curent ros time
	current_pos.header.seq = target_seq;
	current_pos.header.frame_id = "1";
	current_pos.header.stamp = ros::Time::now();

	current_pos_pub.publish(current_pos);
	target_seq++;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gazebo_vicon_remap_node");
    ros::NodeHandle nh("~"); //Node handle for this node

    std::string quad_name ;

    if(!nh.getParam("quad_name",quad_name)){
    	quad_name = "Quad8";
    	ROS_WARN("No quad name provided. Default quad name is %s",quad_name.c_str());
    }

    int remap_freq;
    if(!nh.getParam("frequency",remap_freq)){
    	remap_freq = SENDING_DATA_RATE;
    	ROS_WARN("No remap frequency provided. Default value is %d",remap_freq);
    }

	//Subscribe to the appropriate vicon topics for position and orientation
    ros::Subscriber vicon_sub =nh.subscribe<geometry_msgs::PoseStamped>(("/"+quad_name+"/mavros/local_position/pose").c_str(),1,vicon_callback);
    

	//Sending data to the mocap/pose for mocap
    current_pos_pub = nh.advertise<geometry_msgs::TransformStamped>(("/vicon/"+quad_name+"/"+quad_name).c_str(),1);

    ros::Rate rate(remap_freq); // SENDING_DATA_RATE Hz as the frequency of sending data to PX4

    ROS_WARN("SENDING LOCAL_POS TO VICON of %s POSITION TO PX4 STARTED !!!!!",quad_name.c_str());

    while (ros::ok()){

    	ros::spinOnce();

    	rate.sleep();
    }
    ROS_WARN("SENDING LOCAL_POS TO VICON of %s POSITION NODE STOPPED !!!!!",quad_name.c_str());

}