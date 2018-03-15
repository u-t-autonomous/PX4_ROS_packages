#!/usr/bin/env python
import sys
sys.path.insert( 0 , '~/catkin_ws/src/offboard_control/src')


import rospy
from qcontrol_defs.msg import *
from utils_functions import *
from geometry_msgs.msg import Point

if __name__ == "__main__":

    rospy.init_node('example_offboard_poscontrol')

    # Should be the prefix added to the offboard control node (prefix of all mavros topics)
    # for example "" or "/Quad9" or "/Quad10"
    quad_ros_namespace = ""

    # Create a publisher to send target positions
    send_pos_pub = rospy.Publisher(quad_ros_namespace + '/qcontrol/pos_control', PosControl , queue_size=10)
    
    # Start position control with taking off at the beginning
    start_pos_control(quad_name= quad_ros_namespace , takeoff_before= True)

    #Some destinations points
    target1 = Point(0 , 0 , 4)
    target2 = Point(-3 , 2 , 1) 
    #...

    pos_rate = rospy.Rate(0.2) # wait 5 seconds
    #Send target1
    pos_msg = PosControl()
    pos_msg.pos = target1
    pos_msg.yaw = 0
    send_pos_pub.publish(pos_msg)
    pos_rate.sleep()

    pos_rate = rospy.Rate(0.1) # wait 10 seconds
    #send target2
    pos_msg = PosControl()
    pos_msg.pos = target2
    pos_msg.yaw = 0
    send_pos_pub.publish(pos_msg)
    pos_rate.sleep()

    # Land the vehicule
    start_landing(quad_name= quad_ros_namespace)