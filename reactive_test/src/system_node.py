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
from simpleController import SquareCtrl
from threading import Thread, Lock

X_NUMBER_TILE = 8
Y_NUMBER_TILE = 8

Y_MINIMUM = -1.5
X_MINIMUM = -1.5

Y_MAXIMUM = 1.5
X_MAXIMUM = 1.5

Z_LEVEL = 1.4


ENV_NAME = "/Quad10"
SYS_NAME = "/Quad9"

SYS_INIT_X = 0
SYS_INIT_Y = 0

ONE_MOVE_DURATION = 2
SEND_POS_FREQ = 5

threadLock = Lock()

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


class Env_node(Thread):
	def __init__(self,quad_name,grid , quad, publisher):
		Thread.__init__(self)
		self.grid = grid
		self.quad = quad
		self.action = "stay"
		self.publisher = publisher
		self.is_posctl = False
		self.quad_name = quad_name

	def handle_joy(self,msg):
		if msg.buttons[13] == 1 :
			self.action = "forward"
		elif msg.buttons[14] == 1 :
			self.action = "backward"
		elif msg.buttons[12] == 1 :
			self.action = "right"
		elif msg.buttons[11] == 1 :
			self.action = "left"
		elif msg.buttons[0] == 1:
	        #A button
			self.action = "stay"
		elif msg.buttons[3] == 1 :
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

	def move(self):
		quad_pos = self.quad.get_current_pos()
		new_state = self.grid.vicon2state(quad_pos)
		if(self.action == "forward"):
			new_state = new_state+X_NUMBER_TILE
		elif (self.action == "backward"):
			new_state = new_state-X_NUMBER_TILE
		elif (self.action == "left"):
			new_state = new_state+1
		elif (self.action == "right"):
			new_state = new_state-1
		next_position = self.grid.state2vicon(new_state)
		if(self.is_posctl):
			move_sys(quad_pos,next_position,SEND_POS_FREQ,ONE_MOVE_DURATION,self.publisher)

	def run(self):
		while not rospy.is_shutdown():
			quad_pos = self.quad.get_current_pos()
			new_state = self.grid.vicon2state(quad_pos)
			if(self.action == "forward"):
				new_state = new_state+X_NUMBER_TILE
			elif (self.action == "backward"):
				new_state = new_state-X_NUMBER_TILE
			elif (self.action == "left"):
				new_state = new_state+1
			elif (self.action == "right"):
				new_state = new_state-1
				next_position = self.grid.state2vicon(new_state)
				if(self.is_posctl):
					move_sys(quad_pos,next_position,SEND_POS_FREQ,ONE_MOVE_DURATION,self.publisher)

class Sys_node(Thread):
	def __init__(self,sys_name,grid,quad,publisher,current_controller):
		Thread.__init__(self)
		self.grid = grid
		self.sys_name = sys_name
		self.quad = quad
		self.publisher = publisher
		self.current_controller = current_controller

	def sys_quad_init(self):
		rospy.wait_for_service(self.sys_name+'/qcontrol/commands')
		cmdAction = CommandActionRequest()
		cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
		cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
		cmd_srv = rospy.ServiceProxy(self.sys_name+'/qcontrol/commands', CommandAction)
		reponse = cmd_srv(cmdAction)
		next_state = self.current_controller.move(self.quad.grid_state(self.grid))['loc']
		next_move = self.grid.state2vicon(next_state)
		print ("current state: ",self.grid.vicon2state(self.grid.get_position()),"NEXT_MOVE: ",next_move, "Next State: ",next_state," obstacle state: ",self.quad.grid_state(self.grid))
		move_sys(self.grid.get_position(),next_move,SEND_POS_FREQ,15,self.publisher)

	def move(self):
		res = self.current_controller.move(self.quad.grid_state(self.grid))
		next_state , stage = res['loc'], res['stage']
		next_move = self.grid.state2vicon(next_state)
		print ("current state: ",self.grid.vicon2state(self.grid.get_position()),"NEXT_MOVE: ",next_move, "Next State: ",next_state," obstacle state: ",self.quad.grid_state(grid),"current stage : ",stage)
		move_sys(self.grid.get_position(),next_move,SEND_POS_FREQ,ONE_MOVE_DURATION,self.publisher)

	def run(self):
		self.sys_quad_init()
		while not rospy.is_shutdown():
			res = self.current_controller.move(self.quad.grid_state(self.grid))
			next_state , stage = res['loc'], res['stage']
			next_move = self.grid.state2vicon(next_state)
			print ("current state: ",self.grid.vicon2state(self.grid.get_position()),"NEXT_MOVE: ",next_move, "Next State: ",next_state," obstacle state: ",self.quad.grid_state(grid),"current stage : ",stage)
			move_sys(self.grid.get_position(),next_move,SEND_POS_FREQ,ONE_MOVE_DURATION,self.publisher)


class Quad:

	def __init__(self,name):
		self.name = name
		self.current_pos = Point()

	def handle_position(self,msg):
		threadLock.acquire()
		self.current_pos.x = msg.transform.translation.x
		self.current_pos.y = msg.transform.translation.y
		self.current_pos.z = Z_LEVEL
		threadLock.release()

	def grid_state(self,curr_grid):
		threadLock.acquire()
		state = curr_grid.vicon2state(self.current_pos)
		threadLock.release()
		return state

	def get_current_pos(self):
		threadLock.acquire()
		res = self.current_pos
		threadLock.release()
		return res

class Grid:

	def __init__(self, x, y, z, base, maximum):

		self.x = x
		self.y = y
		self.base = base
		self.maximum = maximum
		self.blocklengthX = (maximum.x - base.x)/x
		self.blocklengthY = (maximum.y - base.y)/y
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

def send_trajectory(traj_3d, sampling_rate,publisher):
	r = rospy.Rate(sampling_rate)
	for i in range(len(traj_3d.time)):
		att_msg = AttPVA()
		att_msg.use_position = True
		att_msg.use_speed = True
		att_msg.use_acceleration = False
		att_msg.use_rate = False
		att_msg.use_yaw = True

		att_msg.yaw = 0
		att_msg.posZ_thrust =traj_3d.position[i].position.z
		att_msg.posY_pitch = traj_3d.position[i].position.y
		att_msg.posX_roll = traj_3d.position[i].position.x

		att_msg.velZ=traj_3d.velocity[i].linear.z
		att_msg.velY=traj_3d.velocity[i].linear.y
		att_msg.velX=traj_3d.velocity[i].linear.x

		att_msg.accZ=traj_3d.acceleration[i].force.z
		att_msg.accY=traj_3d.acceleration[i].force.y
		att_msg.accX=traj_3d.acceleration[i].force.x
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

def move_sys(current_pos,next_pos,freq,duration,publisher):
	x_inputs = {'p_init' : current_pos.x, 'p_final' : next_pos.x, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	y_inputs = {'p_init' : current_pos.y, 'p_final' : next_pos.y, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	z_inputs = {'p_init' : current_pos.z, 'p_final' : next_pos.z, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
	traj_x = get_traj(**x_inputs)
	traj_y = get_traj(**y_inputs)
	traj_z = get_traj(**z_inputs)
	traj_3d = make_3d_traj(traj_x, traj_y, traj_z)
	send_trajectory(traj_3d,freq,publisher)


if __name__ == "__main__":

	current_controller = SquareCtrl()

	#Grid definition
	base = Point()
	base.x = X_MINIMUM
	base.y = Y_MINIMUM
	base.z = Z_LEVEL
	maximum = Point()
	maximum.x = X_MAXIMUM
	maximum.y = Y_MAXIMUM

	grid = Grid(X_NUMBER_TILE, Y_NUMBER_TILE,Z_LEVEL, base, maximum)
	quad_env = Quad("env1")

	#Gazebo test
	#quad_env.current_pos = grid.state2vicon(24)

	rospy.init_node('system_node', anonymous=True)
	rospy.Subscriber("/vicon"+SYS_NAME+SYS_NAME, TransformStamped, grid.handle_position)
	rospy.Subscriber("/vicon"+ENV_NAME+ENV_NAME, TransformStamped, quad_env.handle_position)
	#rospy.Subscriber(SYS_NAME+"/mavros/local_position/pose", PoseStamped, grid.handle_position)

	sys_traj_pub = rospy.Publisher(SYS_NAME+"/qcontrol/att_pva", AttPVA, queue_size=1)
	env_traj_pub = rospy.Publisher(ENV_NAME+"/qcontrol/att_pva", AttPVA, queue_size=1)

	env_thread = Env_node(ENV_NAME,grid,quad_env,env_traj_pub)
	sys_thread = Sys_node(SYS_NAME,grid,quad_env,sys_traj_pub,current_controller)


	rospy.Subscriber("/joy",Joy,env_thread.handle_joy)

	wait_rate = rospy.Rate(50)

	#Starting environment thread
	#env_thread.start()
	while not env_thread.is_posctl:
		print ("POSCTL waiting !!!!")
		wait_rate.sleep()

	sys_thread.sys_quad_init()
	while not rospy.is_shutdown():
		sys_thread.move()
		env_thread.move()
		wait_rate.sleep()
    #Starting system thread
    #sys_thread.start()

    #env_thread.join()
    #sys_thread.join()

    #wait_rate.sleep()
    #Moving the sys to the initial state
    #sys_quad_init()
    #wait_rate.sleep()
    # while not rospy.is_shutdown():
    #     #next_state = current_controller.move(quad_env.grid_state(grid)%X_NUMBER_TILE)['loc']
    #     res = current_controller.move(quad_env.grid_state(grid))
    #     next_state , stage = res['loc'], res['stage']
    #     next_move = grid.state2vicon(next_state)
    #     print ("current state: ",grid.vicon2state(grid.get_position()),"NEXT_MOVE: ",next_move, "Next State: ",next_state," obstacle state: ",quad_env.grid_state(grid),"current stage : ",stage)
    #     move_sys(grid.get_position(),next_move,SEND_POS_FREQ,ONE_MOVE_DURATION,sys_traj_pub)
    #     #wait_rate.sleep()