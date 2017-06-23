#!/usr/bin/env python
import sys
import rospy
from quad_control.srv import *
import time
import re
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from quad_control.msg import TrajArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from qcontrol_defs.msg import PVA
from qcontrol_defs.msg import PVAStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from qcontrol_defs.msg import AttPVA
from qcontrol_defs.msg import QuadState
from qcontrol_defs.srv import *
from time import sleep
import numpy as np
from time import sleep
from math import floor

from sensor_msgs.msg import Joy
from followMeController import FollowMeCtrl
from random import randint


from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

X_NUMBER_TILE = 10
Y_NUMBER_TILE = 10

Y_MINIMUM = -5 #1.5
X_MINIMUM = -5

Y_MAXIMUM = 5
X_MAXIMUM = 5

Z_LEVEL = 1.4


ENV_NAME = "/Quad10"
SYS_NAME = "/Quad9"

ONE_MOVE_DURATION = 8
SEND_POS_FREQ = 10

sys_target_number = 4 
sys_target_point = [22,27,57,72]

env_target_point = [6,15,24,34,44,53,62,72,82,83,84,85,75,65,56,47,37,27,18,8,7]
curr_color = [1.0,0.0,0.0]

def create_marker(i,types,use_text):
	marker = Marker()
	marker.header.frame_id = "1"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "target_cells"
	if use_text :
		marker.id = sys_target_number+i
		marker.text = ""+str(i)
		marker.color.r = 0.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1
		marker.scale.z = 0.3
	else :
		marker.id = i
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 0.5
		marker.scale.z = 0.05

	marker.type = types
	marker.action = Marker.ADD
	cur_pos =  grid.state2vicon(sys_target_point[i])
	marker.pose.position.x = cur_pos.x
	marker.pose.position.y = cur_pos.y
	marker.pose.position.z = 0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = grid.blocklengthX-0.05
	marker.scale.y = grid.blocklengthY-0.05
	return marker

def send_setpoint(init_point,end_point):
	marker = Marker()
	marker.header.frame_id = "1"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "setpoint_target"
	marker.type = Marker.LINE_STRIP
	marker.action = Marker.ADD
	marker.color.r = curr_color[0]
	marker.color.g = curr_color[1]
	marker.color.b = curr_color[2]
	marker.color.a = 1.0
	#marker.scale.x = grid.blocklengthX-0.05
	#marker.scale.y = grid.blocklengthY-0.05
	marker.scale.z = 0.05
	marker.points.append(init_point)
	marker.points.append(end_point)
	set_point_pub.publish(marker)


def send_target_rviz() :
	markers = MarkerArray()
	for i in range(sys_target_number):
		marker = create_marker(i,Marker.CUBE,False)
		marker_text = create_marker(i,Marker.TEXT_VIEW_FACING,True)
		markers.markers.append(marker)
		markers.markers.append(marker_text)
	vis_pub.publish(markers)
	print ("SENDING MARKERS FINISHED")

def takeoff(quad_name):
	rospy.wait_for_service(quad_name+'/qcontrol/commands')
	cmdAction = CommandActionRequest()
	cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
	cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
	reponse = cmd_srv(cmdAction)

def start_position_control(quad_name):
	rospy.wait_for_service(quad_name+'/qcontrol/commands')
	cmdAction = CommandActionRequest()
	cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
	cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
	cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
	reponse = cmd_srv(cmdAction)

def land(quad_name):
	rospy.wait_for_service(quad_name+'/qcontrol/commands')
	cmdAction = CommandActionRequest()
	cmdAction.start_landing = CommandActionRequest.START_LANDING_TRUE;
	cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
	reponse = cmd_srv(cmdAction)


class Env_node :
	def __init__(self,quad_name,quad,grid,target_points):
		self.grid = grid
		self.quad = quad
		self.is_posctl = False
		self.quad_name = quad_name
		self.target_points = target_points
		self.curr_ind = 0

	def handle_joy(self,msg):
		if msg.buttons[3] == 1 :
	        #Y buton land
			land(self.quad_name)
			self.is_posctl = False
		elif msg.buttons[1] == 1 :
	        #B button takeoff
			takeoff(self.quad_name)
			self.is_posctl = False
		elif msg.buttons[2] == 1 :
			start_position_control(self.quad_name)
			self.is_posctl = True
		elif msg.buttons[0] == 1 :
			#send target point to RVIZ
			send_target_rviz()

	def env_quad_init(self,publisher):
		rospy.wait_for_service(self.quad_name+'/qcontrol/commands')
		cmdAction = CommandActionRequest()
		cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
		cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
		cmd_srv = rospy.ServiceProxy(self.quad_name+'/qcontrol/commands', CommandAction)
		cmd_srv(cmdAction)
		quad_pos = self.quad.get_current_pos()
		next_position =  self.grid.state2vicon(self.target_points[self.curr_ind])
		curr_color = [1.0,0,0]
		send_setpoint(quad_pos,next_position)
		move_quad(quad_pos,next_position,SEND_POS_FREQ,5,publisher)
		while self.quad.grid_state(self.grid) != self.target_points[self.curr_ind]:
			wait_rate.sleep()
		self.curr_ind = (self.curr_ind +1)%len(self.target_points)

	def move(self,publisher):
		quad_pos = self.quad.get_current_pos()
		next_position = self.grid.state2vicon(self.target_points[self.curr_ind])
		curr_color = [1.0,0,0]
		send_setpoint(quad_pos,next_position)
		move_quad(quad_pos,next_position,SEND_POS_FREQ,ONE_MOVE_DURATION,publisher)
		self.curr_ind = (self.curr_ind +1)%len(self.target_points)

class Sys_node:
	def __init__(self,sys_name,quad_env,current_controller):
		self.quad_env = quad_env
		self.sys_name = sys_name
		self.current_controller = current_controller

	def sys_quad_init(self,publisher):
		rospy.wait_for_service(self.sys_name+'/qcontrol/commands')
		cmdAction = CommandActionRequest()
		cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
		cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
		cmd_srv = rospy.ServiceProxy(self.sys_name+'/qcontrol/commands', CommandAction)
		cmd_srv(cmdAction)
		next_state = self.current_controller.move(self.quad_env.quad.grid_state(self.quad_env.grid))['loc']
		next_move = self.quad_env.grid.state2vicon(next_state)
		curr_color = [1.0,1.0,1.0]
		send_setpoint(self.quad_env.grid.get_position(),next_move)
		move_quad(self.quad_env.grid.get_position(),next_move,SEND_POS_FREQ,5,publisher)
		while self.quad_env.grid.vicon2state(self.quad_env.grid.get_position()) != next_state:
			wait_rate.sleep()

	def move(self,publisher):
		res = self.current_controller.move(self.quad_env.quad.grid_state(self.quad_env.grid))
		next_state  = res['loc']
		next_move = self.quad_env.grid.state2vicon(next_state)
		curr_color = [1.0,1.0,1.0]
		send_setpoint(self.quad_env.grid.get_position(),next_move)
		print ("Current state = ",self.quad_env.grid.vicon2state(self.quad_env.grid.get_position())," , next_state = ",next_state, " , Env_state = ",self.quad_env.quad.grid_state(self.quad_env.grid))
		move_quad(self.quad_env.grid.get_position(),next_move,SEND_POS_FREQ,ONE_MOVE_DURATION,publisher)

class Quad:

	def __init__(self,name):
		self.name = name
		self.current_pos = Point()

	def handle_position(self,msg):
		self.current_pos.x = msg.transform.translation.x
		self.current_pos.y = msg.transform.translation.y
		self.current_pos.z = Z_LEVEL

	def grid_state(self,curr_grid):
		state = curr_grid.vicon2state(self.current_pos)
		return state

	def get_current_pos(self):
		res = self.current_pos
		return res

class Grid:

	def __init__(self, x, y, z, base, maximum):

		self.x = x
		self.y = y
		self.base = base
		self.maximum = maximum
		self.blocklengthX = (float(maximum.x - base.x))/x
		self.blocklengthY = (float(maximum.y - base.y))/y
		self.drone_pos  = Point()
        #assert( ((self.maximum.x - base.x)/self.blocklength) == self.x and ((self.maximum.y - base.y)/self.blocklength) == self.y ), "Your grid sizes do not make sense! "

	def vicon2state(self, position , sigErr = False):
		new_position = Point()
		try:
			assert(position.x >= self.base.x and position.x <= self.maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, base.x, maximum.x)
		except AssertionError:
			old_val = position.x
			if(position.x < self.base.x):
				position.x = base.x
			if(position.x > self.maximum.x):
				position.x = self.maximum.x
			print "x position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (old_val, base.x, maximum.x, position.x)
    		#position.x = (base.x+ maximum.x)/2.
		try:
			assert(position.y >= self.base.y and position.y <= self.maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, base.y, maximum.y)
		except AssertionError:
			old_val = position.y
			if(position.y < self.base.y):
				position.y = base.y
			if(position.y > self.maximum.y):
				position.y = self.maximum.y
			print "y position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (old_val, base.y, maximum.y, position.y)
		new_position.y = int((position.y - self.base.y)/self.blocklengthY)
		new_position.x = int((position.x - self.base.x)/self.blocklengthX)
		try:
			assert(new_position.x >= 0 and new_position.x <= self.x), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.x, 0, self.x)
			assert(new_position.y >= 0 and new_position.y <= self.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.y, 0, self.y)
		except AssertionError:
			new_position.x = 0
			new_position.y = 0

		state = floor(new_position.y) + (floor(new_position.x)*self.x)
		return int(state)

	def state2vicon(self, state):
		xaxis = state / self.x
		yaxis = state % self.x
		position = Point()
		position.x = self.base.x + (self.blocklengthX/2.) + (xaxis * self.blocklengthX ) 
		position.y = self.base.y + (self.blocklengthY/2.) + (yaxis * self.blocklengthY ) 
		position.z = self.base.z
		return position

	def handle_position(self, msg):
		#self.drone_pos = msg.pose.position
		self.drone_pos.x = msg.transform.translation.x
		self.drone_pos.y = msg.transform.translation.y
		#self.drone_pos.z = msg.transform.translation.z
		self.drone_pos.z = Z_LEVEL

	def get_position(self):
		return self.drone_pos

def get_att_pva(current_target_pos,current_target_vel,current_target_acc):
	att_msg = AttPVA()
	att_msg.use_position = True
	att_msg.use_speed = True
	att_msg.use_acceleration = False
	att_msg.use_rate = False
	att_msg.use_yaw = True

	att_msg.yaw = -0.0
	att_msg.posZ_thrust =current_target_pos.position.z
	att_msg.posY_pitch = current_target_pos.position.y
	att_msg.posX_roll = current_target_pos.position.x

	att_msg.velZ=current_target_vel.linear.z
	att_msg.velY=current_target_vel.linear.y
	att_msg.velX=current_target_vel.linear.x

	att_msg.accZ= current_target_acc.force.z
	att_msg.accY=current_target_acc.force.y
	att_msg.accX=current_target_acc.force.x
	return att_msg

def send_trajectory(traj_3d, sampling_rate,publisher):
	r = rospy.Rate(sampling_rate)
	for i in range(len(traj_3d.time)):
		att_msg = get_att_pva(traj_3d.position[i],traj_3d.velocity[i],traj_3d.acceleration[i])
		publisher.publish(att_msg)
		r.sleep()

def get_traj(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
	rospy.wait_for_service('path_planner')
	try:
		traj_srv = rospy.ServiceProxy('path_planner', Traj)
		resp1 = traj_srv(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq)
		return resp1.trajectory
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def make_3d_traj(x, y, z):
	trajlist = TrajArray()
	for i in range(0,len(x)):
		twist = Twist()
		wrench = Wrench()
		pose = Pose()
		twist.linear.x = x[i].velocity
		twist.linear.y = y[i].velocity
		twist.linear.z = z[i].velocity
		wrench.force.x = x[i].acceleration
		wrench.force.y = y[i].acceleration
		wrench.force.z = z[i].acceleration
		pose.position.x = x[i].position
		pose.position.y = y[i].position
		pose.position.z = z[i].position
		trajlist.time.append(x[i].time)
		trajlist.velocity.append(twist)
		trajlist.acceleration.append(wrench)
		trajlist.position.append(pose)
	return trajlist

def get_traj_3d(current_pos,next_pos,freq,duration):
	x_inputs = {'p_init' : current_pos.x, 'p_final' : next_pos.x, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	y_inputs = {'p_init' : current_pos.y, 'p_final' : next_pos.y, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	z_inputs = {'p_init' : current_pos.z, 'p_final' : next_pos.z, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	traj_x = get_traj(**x_inputs)
	traj_y = get_traj(**y_inputs)
	traj_z = get_traj(**z_inputs)
	return make_3d_traj(traj_x, traj_y, traj_z)

def move_quad(current_pos,next_pos,freq,duration,publisher):
	traj_3d = get_traj_3d(current_pos,next_pos,freq,duration)
	send_trajectory(traj_3d,freq,publisher)

if __name__ == "__main__":

	current_controller = FollowMeCtrl()

	#Grid definition
	base = Point()
	base.x = X_MINIMUM
	base.y = Y_MINIMUM
	base.z = Z_LEVEL
	maximum = Point()
	maximum.x = X_MAXIMUM
	maximum.y = Y_MAXIMUM
	maximum.z = Z_LEVEL

	grid = Grid(X_NUMBER_TILE, Y_NUMBER_TILE,Z_LEVEL, base, maximum)
	print(grid.blocklengthX)
	print(grid.blocklengthY)
	quad_env1 = Quad("env1")

	#Gazebo test
	#quad_env.current_pos = grid.state2vicon(24)

	rospy.init_node('system_node', anonymous=True)
	rospy.Subscriber("/vicon"+SYS_NAME+SYS_NAME, TransformStamped, grid.handle_position)
	rospy.Subscriber("/vicon"+ENV_NAME+ENV_NAME, TransformStamped, quad_env1.handle_position)
	#rospy.Subscriber(SYS_NAME+"/mavros/local_position/pose", PoseStamped, grid.handle_position)

	sys_traj_pub = rospy.Publisher(SYS_NAME+"/qcontrol/att_pva", AttPVA, queue_size=10)
	env_traj_pub = rospy.Publisher(ENV_NAME+"/qcontrol/att_pva", AttPVA, queue_size=10)
	vis_pub = rospy.Publisher("visualization_marker_array",MarkerArray,queue_size=1)
	set_point_pub = rospy.Publisher("visualization_marker",Marker,queue_size=1)

	env_thread = Env_node(ENV_NAME,quad_env1,grid,env_target_point)
	sys_thread = Sys_node(SYS_NAME,env_thread,current_controller)


	rospy.Subscriber("/joy",Joy,env_thread.handle_joy)

	wait_rate = rospy.Rate(1)
	
	#Starting environment thread
	#env_thread.start()
	while not env_thread.is_posctl:
		wait_rate.sleep()

	#Send env to an initial position
	env_thread.env_quad_init(env_traj_pub)
	wait_rate.sleep()
	#send sys to his initial position while env is not moving
	sys_thread.sys_quad_init(sys_traj_pub)
	wait_rate.sleep()

	#Finally start the turned based routine
	while not rospy.is_shutdown():
		env_thread.move(env_traj_pub)
		wait_rate.sleep()
		sys_thread.move(sys_traj_pub)
		wait_rate.sleep()