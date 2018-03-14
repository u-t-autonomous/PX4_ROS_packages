#!/usr/bin/env python

import sys
import rospy
from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import tf

import math

max_x = 1.0
max_y = 1.0
max_z = 1.6

z_plan = 1.4

def circle_trajectory(freq, duration,r):
	x = list()
	y = list()
	t = list()
	for i in range(freq):
		t0 = 2 * math.pi * (1.0/(freq-1)) * duration * i
		x.append(r * math.cos(t0/duration))
		y.append(r * math.sin(t0/duration))
		t.append((1.0/(freq-1)) * duration * i)
	return (t,x,y)

def fermat_spiral(freq , duration):
	x = list()
	y = list()
	t = list()
	for i in range(freq):
		t0 = i * (1.0/(freq-1)) * duration
		x.append((max_x * t0 * t0 * math.cos(t0))/(duration * duration ))
		y.append((max_y * t0 * t0 * math.sin(t0))/ (duration * duration ))
		t.append(t0)
	return (t,x,y)

def conical_helix(freq , duration ):
	x = list()
	y = list()
	z = list()
	t = list()
	for i in range(freq):
		t0 = i * (1.0/(freq-1)) * duration
		x.append( (max_x * t0 * math.cos(20 * t0))/duration)
		y.append( (max_y * t0 * math.sin(20 * t0))/duration)
		z.append( 0.3 + ((t0/duration) * max_z ))
		t.append(t0)
	return (t,x,y,z)

def uniform_helix(freq , duration ):
	x = list()
	y = list()
	z = list()
	t = list()
	for i in range(freq):
		t0 = i * (1.0/(freq-1)) * duration
		x.append( max_x * math.cos(1.0 * t0))
		y.append( max_y * math.sin(1.0 * t0))
		z.append( (t0/duration) * max_z )
		t.append(t0)
	return (t,x,y,z)

def generate_traj(x , y , z , traj_time , freq = 30):
	rospy.wait_for_service('/min_snap_trajectory')
	simple_path_plan = SimplePathPlanRequest()
	# simple_path_plan.x = [current_position.x , init_point.x]
	# simple_path_plan.y = [current_position.y , init_point.y]
	# simple_path_plan.z = [current_position.z , init_point.z]
	simple_path_plan.x = x
	simple_path_plan.y = y
	simple_path_plan.z = z

	simple_path_plan.velx_init = [0.0 , 0.0]
	simple_path_plan.accx_init = [0.0 , 0.0]
	simple_path_plan.vely_init = [0.0 , 0.0]
	simple_path_plan.accy_init = [0.0 , 0.0]
	simple_path_plan.velz_init = [0.0 , 0.0]
	simple_path_plan.accz_init = [0.0 , 0.0]

	simple_path_plan.t = traj_time
	simple_path_plan.freq = freq

	min_snap_traj = rospy.ServiceProxy('/min_snap_trajectory' , SimplePathPlan)
	res = min_snap_traj(simple_path_plan)
	#send the waypoint
	send_waypoint(res.traj.pva , freq)
	return res.traj.pva

def send_trajectory(pva_list , publisher , freq):
	rate = rospy.Rate(freq)
	for elem in pva_list:
		publisher.publish(elem)
		rate.sleep()

def curr_pos_callback(msg):
	global current_position
	current_position = msg.pose.pose.position

def start_pva_control():
	rospy.wait_for_service('/qcontrol/commands')
	cmdAction = CommandActionRequest()
	cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
	cmdAction.is_pvactl = CommandActionRequest.IS_PVACTL_TRUE
	cmd_srv = rospy.ServiceProxy('/qcontrol/commands', CommandAction)
	reponse = cmd_srv(cmdAction)

def send_waypoint(pva_list , freq):
	path = Path()
	path.header.seq = 0
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = 'fcu'
	iter_pos = 0
	curr_time = rospy.Time.now()
	for elem in pva_list:
		pos_stamp = PoseStamped()
		pos_stamp.pose.position = elem.pos
		quater = tf.transformations.quaternion_from_euler(0.0,0.0,elem.yaw)
		pos_stamp.pose.orientation.x = quater[0]
		pos_stamp.pose.orientation.y = quater[1]
		pos_stamp.pose.orientation.z = quater[2]
		pos_stamp.pose.orientation.w = quater[3]
		pos_stamp.header.seq = iter_pos
		pos_stamp.header.frame_id = 'fcu'
		pos_stamp.header.stamp = curr_time + rospy.Duration(1.0/freq)
		path.poses.append(pos_stamp)
	path_publisher.publish(path)

if __name__ == '__main__':

	rospy.init_node('fancy_traj_node', anonymous=True)

	rospy.Subscriber("/mavros/local_position/odom",Odometry , curr_pos_callback)
	pub = rospy.Publisher("/qcontrol/pva_control", PVA , queue_size=10)
	path_publisher = rospy.Publisher('/qcontrol/path_pva', Path  , queue_size=10)

	freq_pub = 30

	current_position = Point()
	start_pva_control()

	while not rospy.is_shutdown():
		chosen_traj = raw_input('[ circle spiral conical_helix uniform_helix ]')
		t = list()
		x = list()
		y = list()
		z = list()
		if chosen_traj == 'circle':
			(t , x , y) = circle_trajectory(30 , 30  , 1.5)
			t = [t[0] , t[len(t)-1]]
			z = [ z_plan for val in range(len(x))]
		elif chosen_traj == 'spiral':
			(t , x , y) = fermat_spiral(30 , 30 )
			t = [t[0] , t[len(t)-1]]
			z = [ z_plan for val in range(len(x))]
		elif chosen_traj == 'conical_helix':
			(t , x , y , z ) = conical_helix(30,30)	
			t = [t[0] , t[len(t)-1]]
		else :
			(t , x , y , z ) = uniform_helix(30,30)
			t = [t[0] , t[len(t)-1]]
		#send initial position
		x_init = [current_position.x , x[0]]
		y_init = [current_position.y , y[0]]
		z_init = [current_position.z , z[0]]
		send_trajectory(generate_traj( x_init, y_init , z_init , [0.0 , 3] , freq_pub) , pub , freq_pub)
		
		#Wait authorization for starting new trajectory
		val = raw_input("Start trajectory ...")
		send_trajectory(generate_traj(x , y , z , t , freq_pub) , pub , freq_pub)


