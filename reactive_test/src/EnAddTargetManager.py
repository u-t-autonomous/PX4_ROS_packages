#!/usr/bin/env python
import sys
import rospy

from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

import tf

from threading import Thread
from threading import Lock

from math import floor

import ModTwoAddTargetController as env_controller
import EnAddTargetController as sys_controller


class Grid:
	#To being able to superpose this with tulip and the ENU coordinate system
	# x here represent X in ENU and also the number of row
	# y here represent Y in ENU and also the number of column
	def __init__(self, x, y, base, maximum):

		self.x = x
		self.y = y
		self.base = base
		self.maximum = maximum
		self.blocklengthX = (float(maximum.x - base.x))/x
		self.blocklengthY = (float(maximum.y - base.y))/y

	def vicon2state(self, position):
		new_position = Point()
		assert(position.x >= self.base.x and position.x <= self.maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, self.base.x, self.maximum.x)
		assert(position.y >= self.base.y and position.y <= self.maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, self.base.y, self.maximum.y)
		new_position.y = int((position.y - self.base.y)/self.blocklengthY)
		new_position.x = int((position.x - self.base.x)/self.blocklengthX)
		assert(new_position.x >= 0 and new_position.x <= self.x), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.x, 0, self.x)
		assert(new_position.y >= 0 and new_position.y <= self.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.y, 0, self.y)
		state = floor(new_position.y) + (floor(new_position.x)*self.y)
		return int(state)

	def state2vicon(self, state):
		xaxis = state / self.y
		yaxis = state % self.y
		position = Point()
		position.x = self.base.x + (self.blocklengthX/2.) + (xaxis * self.blocklengthX ) 
		position.y = self.base.y + (self.blocklengthY/2.) + (yaxis * self.blocklengthY ) 
		position.z = self.base.z
		return position

	def vicon2state_list(self , position):
		res = list()
		for elem in position:
			res.append(self.vicon2state(elem))
		return res
		
def generate_traj(x , y , traj_time , corr=None , freq = 20 , vel_init = [0.0,0.0] , acc_init=[0.0,0.0]):
	rospy.wait_for_service('/min_snap_trajectory')
	simple_path_plan = SimplePathPlanRequest()
	simple_path_plan.x = x
	simple_path_plan.y = y
	#simple_path_plan.z = z

	simple_path_plan.velx_init = [vel_init[0] , 0.0]
	simple_path_plan.accx_init = [acc_init[0] , 0.0]
	simple_path_plan.vely_init = [vel_init[1] , 0.0]
	simple_path_plan.accy_init = [acc_init[1] , 0.0]
	#simple_path_plan.velz_init = [0.0 , 0.0]
	#simple_path_plan.accz_init = [0.0 , 0.0]

	#Set corridors constraints if there exist any
	if not corr is None:
		for elem in range(len(x)-1):
			simple_path_plan.corridors.append(corr)

	#Set Time and frequency 
	simple_path_plan.t = traj_time
	simple_path_plan.freq = freq

	#connect to the server and send the request
	min_snap_traj = rospy.ServiceProxy('/min_snap_trajectory' , SimplePathPlan)
	res = min_snap_traj(simple_path_plan)
	#send the waypoint
	#send_waypoint(res.traj.pva , freq)
	for elem in res.traj.pva:
		elem.pos.z = Z_LEVEL
	return res.traj

#Take a pva list and transform it to a nav_msg.Path for RVIZ visualization
def send_waypoint(pva_list , path_pub ,ind_seq = 0):
	path = Path()
	path.header.seq = ind_seq
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = 'fcu'
	iter_pos = 0
	curr_time = rospy.Time.now()
	curr_freq = pva_list.wait_freq[0]
	for elem in pva_list.pva:
		if elem == pva_list.pva[len(pva_list.pva)-1]:
			continue
		pos_stamp = PoseStamped()
		pos_stamp.pose.position = elem.pos
		pos_stamp.pose.position.z = Z_LEVEL
		quater = tf.transformations.quaternion_from_euler(0.0,0.0,elem.yaw)
		pos_stamp.pose.orientation.x = quater[0]
		pos_stamp.pose.orientation.y = quater[1]
		pos_stamp.pose.orientation.z = quater[2]
		pos_stamp.pose.orientation.w = quater[3]
		pos_stamp.header.seq = iter_pos
		pos_stamp.header.frame_id = 'fcu'
		pos_stamp.header.stamp = curr_time + rospy.Duration(1.0/curr_freq)
		path.poses.append(pos_stamp)
	path_pub.publish(path)

#Client request to offboard server for arming motors and going into PVA control mode
def start_pva_control(quad_name , takeoff_before= False):
	rospy.wait_for_service('/' + quad_name + '/qcontrol/commands')
	cmd_srv = rospy.ServiceProxy('/' + quad_name + '/qcontrol/commands', CommandAction)
	takeoff_wait = rospy.Rate(0.2)	# We wait for 5 second before going in PVA control mode in case takeoff is asked
	if takeoff_before :
		cmdAction = CommandActionRequest()
		cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
		cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
		reponse =  cmd_srv(cmdAction)
		takeoff_wait.sleep()
	cmdAction = CommandActionRequest()
	cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
	cmdAction.is_pvactl = CommandActionRequest.IS_PVACTL_TRUE
	reponse = cmd_srv(cmdAction)
	takeoff_wait.sleep()


# change a list of state to it's appropriate Position object
def from_state_to_pva(list_state , grid_):
	res = list()
	for state in list_state:
		position = grid_.state2vicon(state)
		res.append(position)
	return res

def get_occupancy_grid(msg):
	global occupancy_grid
	occupancy_grid = msg

def get_clicked_point(msg):
	global last_clicked_point
	global is_static
	is_static = True
	last_clicked_point = msg.pose.pose.position

def publish_user_setpoint(curr_position , m_color=[1.0,0.0,0.0,1.0] , cube_size=0.6):
	global marker_id
	marker = Marker()
	marker.header.frame_id = "fcu"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "setpoint_target"
	marker.id = marker_id
	marker.type = Marker.CUBE
	marker.action = Marker.ADD
	marker.color.r = m_color[0]
	marker.color.g = m_color[1]
	marker.color.b = m_color[2]
	marker.color.a = m_color[3]
	marker.scale.x = cube_size
	marker.scale.y = cube_size
	marker.scale.z = 0.01
	marker.pose.position = curr_position
	marker.pose.orientation.w = 1.0
	click_point_publisher.publish(marker)
	marker_id = marker_id +1


def get_information_quad(ind_quad):
	quad_name = raw_input('Enter name of the quad_{ind_quad} : '.format(ind_quad= ind_quad))
	waypoints = list()
	number_setpoint = int(raw_input('Number of setpoint : '))
	for elem in range(number_setpoint):
		raw_input(' Click on a target cell/Point and then press enter !')
		waypoints.append(last_clicked_point)
		publish_user_setpoint(grid.state2vicon(grid.vicon2state(last_clicked_point)))
		print grid.vicon2state(last_clicked_point)
	return (quad_name , waypoints)

def pva_callback(msg , set_point_pub):
	global msg_count
	pos = PoseStamped()
	pos.header.seq = msg_count
	pos.header.frame_id ='fcu'
	pos.header.stamp = rospy.Time.now()
	pos.pose.position = msg.pos
	pos.pose.position.z = Z_LEVEL
	quater = tf.transformations.quaternion_from_euler(0.0,0.0,msg.yaw)
	pos.pose.orientation.x = quater[0]
	pos.pose.orientation.y = quater[1]
	pos.pose.orientation.z = quater[2]
	pos.pose.orientation.w = quater[3]
	set_point_pub.publish(pos)
	msg_count = msg_count +1

def quad9_odometry(msg):
	global quad9_pos
	quad9_pos = msg.pose.pose.position


def quad10_odometry(msg):
	global quad10_pos
	quad10_pos = msg.pose.pose.position

def quad8_odometry(msg):
	global quad8_pos
	quad8_pos = msg.pose.pose.position


if __name__ == "__main__":

	rospy.init_node('static_emvironment_node')

	#Get some subscribers to the occupancy grid and click point for human target set point
	rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped ,  get_clicked_point)
	rospy.Subscriber('/Quad9/mavros/local_position/odom', Odometry , quad9_odometry)
	rospy.Subscriber('/Quad10/mavros/local_position/odom', Odometry , quad10_odometry)
	rospy.Subscriber('/Quad8/mavros/local_position/odom', Odometry , quad8_odometry)

	#USER setpoint target publisher
	click_point_publisher = rospy.Publisher('/user_target_point' , Marker , queue_size=10)
	set_point_pub_quad9 = rospy.Publisher('Quad9/qcontrol/pva_setpoint', PoseStamped  , queue_size=10)
	set_point_pub_quad10 = rospy.Publisher('Quad10/qcontrol/pva_setpoint', PoseStamped ,  queue_size=10)
	set_point_pub_quad8 = rospy.Publisher('Quad8/qcontrol/pva_setpoint', PoseStamped ,  queue_size=10)

	#Initialise PVA setpoint publisher for the different quad -- Also initialize all systems specifications
	pva_pub_quad9 = rospy.Publisher('/Quad9/qcontrol/pva_control' , PVA , queue_size=10)
	pva_pub_quad10 = rospy.Publisher('/Quad10/qcontrol/pva_control' , PVA , queue_size=10)
	pva_pub_quad8 = rospy.Publisher('/Quad8/qcontrol/pva_control' , PVA , queue_size=10)

	path_pub_quad9 = rospy.Publisher('/Quad9/qcontrol/path_pva' , Path , queue_size=10)
	path_pub_quad10 = rospy.Publisher('/Quad10/qcontrol/path_pva' , Path , queue_size=10)
	path_pub_quad8 = rospy.Publisher('/Quad8/qcontrol/path_pva' , Path , queue_size=10)

	#global occupancy_grid , last clicked point variable
	last_clicked_point = Point()
	marker_id = 0
	msg_count =0
	quad9_pos = Point()
	quad10_pos = Point()
	quad8_pos = Point()

	#Read all needed information from a file 
	filename = raw_input('Path to the file with settings : ')
	show_occupancy_grid = int(raw_input('Show occupancy grid : (1 for Yes and 0 for No) '))
	Z_LEVEL = float(raw_input('Z axis Level = '))

	settings_file = open(filename+'.txt' , 'r')

	#Number of row and number of column and resolution
	(X_NUMBER_TILE_s , Y_NUMBER_TILE_s , resolution_s) =  settings_file.readline().split(";")
	(X_NUMBER_TILE , Y_NUMBER_TILE , resolution) =  (int(X_NUMBER_TILE_s) , int(Y_NUMBER_TILE_s) , float(resolution_s))

	#Get the base point of the grid and deduce maximum point
	(base_x_s , base_y_s) = settings_file.readline().split(";")
	(base_x , base_y ) = (float(base_x_s) , float(base_y_s))

	base = Point()
	base.x = base_x
	base.y = base_y
	base.z = Z_LEVEL

	maxi = Point()
	maxi.x = resolution * X_NUMBER_TILE + base.x
	maxi.y = resolution * Y_NUMBER_TILE + base.y
	maxi.z = Z_LEVEL

	#Get Starting state
	(initial_state_sys_s , initial_state_env_s) = settings_file.readline().split(";")
	(initial_state_sys , initial_state_env) = (int(initial_state_sys_s) , int(initial_state_env_s))

	#Get original target setpoint
	waypoints_state_sys = [int(state) for state in settings_file.readline().split(";")]
	waypoints_state_env = [int(state) for state in settings_file.readline().split(";")]

	#Get Possible target setpoint
	possible_target_sys = [int(state) for state in settings_file.readline().split(";")]
	possible_target_env = [int(state) for state in settings_file.readline().split(";")]

	#Get the static obstacles
	new_static_obstacles = set()
	static_obstacles = set()
	for row in range(X_NUMBER_TILE):
		row_string = settings_file.readline().split(";")
		for col in range(Y_NUMBER_TILE):
			if int(row_string[col]) == 1:
				static_obstacles.add((X_NUMBER_TILE-1-row)* Y_NUMBER_TILE + (Y_NUMBER_TILE-1-col))

	# print static_obstacles
	# print waypoints_state_sys
	# print waypoints_state_env
	# print possible_target_sys
	# print possible_target_env

	#Initialise the grid object
	grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)

	#Publish occupancy grid if asked
	occupancy_wait = rospy.Rate(20)
	if show_occupancy_grid == 1 :
		for elem in (static_obstacles - new_static_obstacles):
			publish_user_setpoint(grid.state2vicon(elem) , [0.0 , 0.0 , 0.0 , 1.0] , resolution)
			occupancy_wait.sleep()
		for elem in new_static_obstacles:
			publish_user_setpoint(grid.state2vicon(elem) , [0.8 , 0.1 , 0.0 , 0.2] , resolution)
			occupancy_wait.sleep()
	else:
		for elem in new_static_obstacles:
			publish_user_setpoint(grid.state2vicon(elem) , [0.8 , 0.1 , 0.0 , 0.2] , resolution)
			occupancy_wait.sleep()

	for elem in waypoints_state_sys:
		publish_user_setpoint(grid.state2vicon(elem) , [1.0 , 1.0 , 1.0 , 0.6] , resolution)
		occupancy_wait.sleep()

	for elem in possible_target_sys:
		publish_user_setpoint(grid.state2vicon(elem) , [0.0,1.0,0.0,0.3] , resolution/1.5)
		occupancy_wait.sleep()

	for elem in waypoints_state_env:
		publish_user_setpoint(grid.state2vicon(elem) , [0.0 , 0.0 , 1.0 , 0.6] , resolution)
		occupancy_wait.sleep()

	for elem in possible_target_env:
		publish_user_setpoint(grid.state2vicon(elem) , [0.0,1.0,1.0,0.3] , resolution/1.5)
		occupancy_wait.sleep()

	#publish_user_setpoint(grid.state2vicon(initial_state_sys) , [1.0 , 0.0 , 0.0 , 1.0] , resolution)
	#occupancy_wait.sleep()
	#publish_user_setpoint(grid.state2vicon(initial_state_env) , [1.0 , 0.0 , 0.0 , 1.0] , resolution)

	#Create the controller and object and synthesize it if it is not already the case
	agent_controller_sys = sys_controller.AgentController(initial_state_sys,initial_state_env ,filename+'_sys', X_NUMBER_TILE , Y_NUMBER_TILE ,waypoints_state_sys , "Quad9" , possible_target_sys, static_obstacles)
	agent_controller_env = env_controller.AgentController(initial_state_env,filename+'_env', X_NUMBER_TILE , Y_NUMBER_TILE ,waypoints_state_env , "Quad10" , possible_target_env, static_obstacles)
	controller_sys = agent_controller_sys.getController()
	controller_env = agent_controller_env.getController()

	######################################################################################################
	#Some takeoff procedure and going to the initial state
	start_pva_control('Quad9',True)
	start_pva_control('Quad8',True)
	start_pva_control('Quad10',True)
	#####################################################################################################
	
	raw_input('wait quad ... ')
	last_env_position = initial_state_env
	segment_duration = 3 #In second
	freq_sol = 50
	ind_spec = 0

	pub_wait = rospy.Rate(freq_sol)

	next_target_state_sys = len(possible_target_sys)
	next_target_state_env = len(possible_target_env)

	next_move_env = controller_env.move(ind_spec , next_target_state_env)
	next_move_sys = controller_sys.move(initial_state_env , ind_spec, next_target_state_sys )

	#quad9_pos  = grid.state2vicon(next_move_sys['Quad9'])
	#quad10_pos = grid.state2vicon(last_env_position)
	#quad8_pos = grid.state2vicon(ind_spec)

	next_pos_sys = grid.state2vicon(next_move_sys['Quad9'])
	next_pos_env = grid.state2vicon(next_move_env['Quad10'])
	next_pos_spec = grid.state2vicon(ind_spec)
	res_traj_sys = generate_traj([quad9_pos.x, next_pos_sys.x],[quad9_pos.y, next_pos_sys.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])
	res_traj_env = generate_traj([quad10_pos.x, next_pos_env.x],[quad10_pos.y, next_pos_env.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])
	res_traj_spec = generate_traj([quad8_pos.x, next_pos_spec.x],[quad8_pos.y, next_pos_spec.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])

	send_waypoint(res_traj_sys , path_pub_quad9)
	send_waypoint(res_traj_env , path_pub_quad10)
	send_waypoint(res_traj_spec , path_pub_quad8)

	# raw_input('send init point ... ')
	for i in range(len(res_traj_sys.pva)):
		#pva_callback(res_traj_sys.pva[i] , set_point_pub_quad9)
		#pva_callback(res_traj_env.pva[i] , set_point_pub_quad10)
		#pva_callback(res_traj_spec.pva[i] , set_point_pub_quad8)
		pva_pub_quad9.publish(res_traj_sys.pva[i])
		pva_pub_quad10.publish(res_traj_env.pva[i])
		pva_pub_quad8.publish(res_traj_spec.pva[i])
		pub_wait.sleep()

	next_target_state_sys = 0
	next_target_state_env = 0
	raw_input('Ready ... ')
	coeff = 1
	while not rospy.is_shutdown():
		if ind_spec == X_NUMBER_TILE-1:
			coeff = -1
		elif ind_spec == 0:
			coeff = 1

		#quad8_pos = grid.state2vicon(ind_spec * (X_NUMBER_TILE +1))
		ind_spec += coeff
		last_next_state_sys = next_target_state_sys
		last_next_state_env = next_target_state_env
		sys_target_changed = False
		env_target_changed = False
		last_sys_position = next_move_sys['Quad9']
		#quad9_pos  = grid.state2vicon(next_move_sys['Quad9'])
		#quad10_pos = grid.state2vicon(last_env_position)

		try:
			next_target_state_sys = possible_target_sys.index(grid.vicon2state(last_clicked_point))
		except AssertionError:
			next_target_state_sys = len(possible_target_sys)
		except ValueError:
			next_target_state_sys = len(possible_target_sys)

		if ( (next_target_state_sys == len(possible_target_sys))):
			next_target_state_sys = last_next_state_sys
		elif next_target_state_sys != last_next_state_sys:
			sys_target_changed = True

		try:
			next_target_state_env = possible_target_env.index(grid.vicon2state(last_clicked_point))
		except AssertionError:
			next_target_state_env = len(possible_target_env)
		except ValueError:
			next_target_state_env = len(possible_target_env)

		if ( (next_target_state_env == len(possible_target_env))):
			next_target_state_env = last_next_state_env
		elif next_target_state_env != last_next_state_env:
			env_target_changed = True

		if env_target_changed:
			next_move_env = controller_env.move(ind_spec , len(possible_target_env))
		else:
			next_move_env = controller_env.move(ind_spec , next_target_state_env)

		iter = 5
		while next_move_env['Quad10'] == last_env_position and (iter >=0) :
			next_move_env = controller_env.move(ind_spec , next_target_state_env)
			iter = iter -1

		if sys_target_changed:
			next_move_sys = controller_sys.move(last_env_position , ind_spec ,  len(possible_target_sys))
			iter = 5
			while (next_move_sys['Quad9'] == last_sys_position) and (iter >= 0) :
				next_pos_sys = controller_sys.move(last_env_position,ind_spec,next_target_state_sys )
				iter = iter-1
		else:
			next_move_sys = controller_sys.move(last_env_position , ind_spec, next_target_state_sys)

		if next_move_env['Quad10'] == last_env_position and next_move_sys['Quad9'] == last_sys_position:
			continue 
		
		last_env_position = next_move_env['Quad10']
		next_pos_sys = grid.state2vicon(next_move_sys['Quad9'])
		next_pos_env = grid.state2vicon(last_env_position)
		next_pos_spec = grid.state2vicon((ind_spec)*(X_NUMBER_TILE +1))
		res_traj_sys = generate_traj([quad9_pos.x, next_pos_sys.x],[quad9_pos.y, next_pos_sys.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])
		res_traj_env = generate_traj([quad10_pos.x, next_pos_env.x],[quad10_pos.y, next_pos_env.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])
		res_traj_spec = generate_traj([quad8_pos.x, next_pos_spec.x],[quad8_pos.y, next_pos_spec.y],[0.0 , segment_duration],freq=freq_sol,corr=None,vel_init=[0.0,0.0] , acc_init=[0.0,0.0])
		
		send_waypoint(res_traj_sys , path_pub_quad9)
		send_waypoint(res_traj_env , path_pub_quad10)
		send_waypoint(res_traj_spec , path_pub_quad8)

		for i in range(len(res_traj_sys.pva)):
			#pva_callback(res_traj_sys.pva[i] , set_point_pub_quad9)
			#pva_callback(res_traj_env.pva[i] , set_point_pub_quad10)
			#pva_callback(res_traj_spec.pva[i] , set_point_pub_quad8)

			pva_pub_quad8.publish(res_traj_spec.pva[i])
			pva_pub_quad9.publish(res_traj_sys.pva[i])
			pva_pub_quad10.publish(res_traj_env.pva[i])
			pub_wait.sleep()
