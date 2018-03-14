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

from visualization_msgs.msg import Marker

import tf

from threading import Thread
from threading import Lock

from math import floor

import TwoAddTargetController as tulip_controller

REAL_ALTITUDE = 4.3

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
		pos_stamp.pose.position.z = REAL_ALTITUDE #Bonne altitude
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

# Function for reducing the number of point in a list of discrete state path.
# For example --> consecutive same direction state will be reduce to the init point and end point 
def reduce_state(list_state , trans_duration , row_number , col_number):
	res_t = []
	res_state = [list_state[0]]
	i = 0
	while i < len(list_state)-1:
		curr_row_i = list_state[i]/col_number
		curr_col_i = list_state[i] % col_number
		curr_row_i_1 = list_state[i+1]/col_number
		curr_col_i_1 = list_state[i+1] % col_number
		diff_row = curr_row_i_1 - curr_row_i
		diff_col = curr_col_i_1 - curr_col_i
		k = 1
		for j in range(i+2 , len(list_state)):
			curr_row_j = list_state[j]/col_number
			curr_col_j = list_state[j] % col_number
			diff_row_j = curr_row_j - curr_row_i_1
			diff_col_j = curr_col_j - curr_col_i_1
			if diff_row == diff_row_j and diff_col == diff_col_j:
				k = k+ 1
				curr_row_i_1 = curr_row_j
				curr_col_i_1 = curr_col_j
			else:
				break
		res_state.append(list_state[i+k])
		res_t.append((i+k)*trans_duration)
		#res_state.append(list_state[i+k])
		i =  i + k
	return res_state , res_t

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

def pva_callback(msg):
	global msg_count
	pos = PoseStamped()
	pos.header.seq = msg_count
	pos.header.frame_id ='fcu'
	pos.header.stamp = rospy.Time.now()
	pos.pose.position = msg.pos
	pos.pose.position.z = REAL_ALTITUDE
	quater = tf.transformations.quaternion_from_euler(0.0,0.0,msg.yaw)
	pos.pose.orientation.x = quater[0]
	pos.pose.orientation.y = quater[1]
	pos.pose.orientation.z = quater[2]
	pos.pose.orientation.w = quater[3]
	set_point_pub.publish(pos)
	msg_count = msg_count +1

class QuadControl (Thread):
	def __init__(self , publisher , path_freq):
		Thread.__init__(self)
		self.pub = publisher
		self.path_freq = path_freq
		self.current_path_1 = list()
		self.current_path_2 = list()
		self.is_path_1 = True
		self.current_ind = 0
		self.m_lock = Lock()

	def reset_target(self , new_target):
		self.m_lock.acquire()
		self.current_path = list()
		self.current_path.extend(new_target)
		self.current_ind = 0
		self.m_lock.release()

	def is_current_ind_equal( self , target_ind):
		self.m_lock.acquire()
		res = self.current_ind == target_ind
		self.m_lock.release()
		return res

	def is_current_list_1(self):
		self.m_lock.acquire()
		res = self.is_path_1
		self.m_lock.release()
		return res

	def run(self):
		quad_wait = rospy.Rate(self.path_freq)
		while not rospy.is_shutdown():
			self.m_lock.acquire()
			if self.is_path_1:
				pva_to_send = self.current_path_1[self.current_ind]
				self.current_ind += 1
				if self.current_ind == len(self.current_path_1):
					self.is_path_1 = False
					self.current_ind = 0
			else:
				pva_to_send = self.current_path_2[self.current_ind]
				self.current_ind += 1
				if self.current_ind == len(self.current_path_2):
					self.is_path_1 = True
					self.current_ind = 0
			self.m_lock.release()
			pva_to_send.pos.z = REAL_ALTITUDE
			self.pub.publish(pva_to_send)
			pva_callback(pva_to_send)
			quad_wait.sleep()

def  closest_point(curr_point , list_target):
	dist = 10000000000.0
	iteration = 0
	res = 0
	for elem in list_target:
		pos_elem = grid.state2vicon(elem)
		dist_elem = ((pos_elem.x - curr_point.x) * (pos_elem.x - curr_point.x)) + ((pos_elem.y -curr_point.y)* (pos_elem.y - curr_point.y))
		if dist_elem < dist:
			dist =  dist_elem
			res = iteration
		iteration+=1
	return res

if __name__ == "__main__":

	rospy.init_node('static_emvironment_node')

	filename = raw_input('Path to the file with settings : ')
	show_occupancy_grid = int(raw_input('Show occupancy grid : (1 for Yes and 0 for No) '))
	Z_LEVEL = float(raw_input('Z axis Level = '))
	quad_name = raw_input('Quad Name : ')

	#Get some subscribers to the occupancy grid and click point for human target set point
	occupancy_sub = rospy.Subscriber('/projected_map', OccupancyGrid , get_occupancy_grid)
	rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped ,  get_clicked_point)


	#USER setpoint target publisher
	click_point_publisher = rospy.Publisher('/user_target_point' , Marker , queue_size=100)
	set_point_pub = rospy.Publisher('Quad9/qcontrol/pva_setpoint', PoseStamped  , queue_size=10)

	#Initialise PVA setpoint publisher for the different quad -- Also initialize all systems specifications
	pva_pub = rospy.Publisher('/' + quad_name + '/qcontrol/pva_control' , PVA , queue_size=10)
	path_pub = rospy.Publisher('/' + quad_name + '/qcontrol/path_pva' , Path , queue_size=10)

	#global occupancy_grid , last clicked point variable
	last_clicked_point = Point()
	marker_id = 0
	msg_count =0

	#Read all needed information from a file 

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
	REAL_ALTITUDE = Z_LEVEL

	maxi = Point()
	maxi.x = resolution * X_NUMBER_TILE + base.x
	maxi.y = resolution * Y_NUMBER_TILE + base.y
	maxi.z = Z_LEVEL

	#Get Starting state
	initial_state = int(settings_file.readline())

	#Get original target setpoint
	waypoints_state = [int(state) for state in settings_file.readline().split(";")]

	#Get Possible target setpoint
	possible_target = [int(state) for state in settings_file.readline().split(";")]

	#Get the static obstacles
	static_obstacles = set()
	for row in range(X_NUMBER_TILE):
		row_string = settings_file.readline().split(";")
		for col in range(Y_NUMBER_TILE):
			if int(row_string[col]) == 1:
				static_obstacles.add((X_NUMBER_TILE-1-row)* Y_NUMBER_TILE + (Y_NUMBER_TILE-1-col))

	#Extend static obstacles with a certain margin
	# margin_size = 1
	# new_static_obstacles = set()
	# for elem in static_obstacles:
	# 	new_static_obstacles.update(set(tulip_controller.get_point_strict_dist(elem , margin_size , X_NUMBER_TILE , Y_NUMBER_TILE)))
	# 	new_static_obstacles.add(elem)
	# new_static_obstacles = new_static_obstacles - static_obstacles
	# static_obstacles.update(new_static_obstacles)

	#Initialise the grid object
	grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)

	#Publish occupancy grid if asked
	occupancy_wait = rospy.Rate(50)
	# if show_occupancy_grid :
	# 	for elem in (static_obstacles - new_static_obstacles):
	# 		publish_user_setpoint(grid.state2vicon(elem) , [0.0 , 0.0 , 0.0 , 1.0] , resolution)
	# 		occupancy_wait.sleep()
	# 	for elem in new_static_obstacles:
	# 		publish_user_setpoint(grid.state2vicon(elem) , [0.8 , 0.1 , 0.0 , 0.2] , resolution)
	# 		occupancy_wait.sleep()
	# else:
	# 	for elem in new_static_obstacles:
	# 		publish_user_setpoint(grid.state2vicon(elem) , [0.8 , 0.1 , 0.0 , 0.2] , resolution)
	# 		occupancy_wait.sleep()

	# for elem in waypoints_state:
	# 	publish_user_setpoint(grid.state2vicon(elem) , [1.0 , 1.0 , 0.0 , 1.0] , resolution)
	# 	occupancy_wait.sleep()

	# for elem in possible_target:
	# 	if elem == possible_target[0]:
	# 		continue
	# 	publish_user_setpoint(grid.state2vicon(elem) , [1.0,1.0,0.0,1.0] , resolution)
	# 	occupancy_wait.sleep()

	# publish_user_setpoint(grid.state2vicon(initial_state) , [0.0 , 0.0 , 1.0 , 1.0] , resolution)

	#Create the controller and object and synthesize it if it is not already the case
	agent_controller = tulip_controller.AgentController(initial_state,filename, X_NUMBER_TILE , Y_NUMBER_TILE ,waypoints_state , quad_name , possible_target, static_obstacles)
	controller = agent_controller.getController()


	######################################################################################################
	#Some takeoff procedure and going to the initial state
	start_pva_control(quad_name , True)
	#####################################################################################################

	freq_sol = 50
	precompute_path_step = 30
	precompute_path_duration = 25.0
	next_target_state = len(possible_target)
	wait_rate = rospy.Rate(freq_sol)
	raw_input('Now start the game :')

	#quadController = QuadControl(pva_pub , freq_sol)

	pos_state = list()
	pos_state.append(grid.state2vicon(controller.move(next_target_state)[quad_name]))
	next_target_state = 0
	for elem in range(precompute_path_step):
		next_moves = controller.move(next_target_state)
		pos_state.append(grid.state2vicon(next_moves[quad_name]))
	res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol,corr=occupancy_grid.info.resolution/4)
	#res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[0.0 , precompute_path_duration],freq=freq_sol,corr=occupancy_grid.info.resolution/2)
	
	send_waypoint(res_traj , path_pub)
	# quadController.current_path_1 = res_traj.pva
	# quadController.start()
	for elem in res_traj.pva:
		pva_callback(elem)
		pva_pub.publish(elem)
		wait_rate.sleep()

	# pos_state = list()
	# for elem in range(precompute_path_step):
	# 	next_moves = controller.move(next_target_state)
	# 	pos_state.append(grid.state2vicon(next_moves[quad_name]))
	# res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol,corr=occupancy_grid.info.resolution/2)
	# quadController.current_path_2 = res_traj.pva

	#last_quad_list = quadController.is_current_list_1()
	next_target_state = 1
	last_clicked_point = grid.state2vicon(next_target_state)
	while not rospy.is_shutdown():
		last_position = pos_state[len(pos_state)-1]
		pos_state = list()
		pos_state.append(last_position)
		last_next_state = next_target_state
		try:
			#next_target_state = possible_target.index(grid.vicon2state(last_clicked_point))
			next_target_state = closest_point(last_clicked_point , possible_target)
			print "No problem = ", next_target_state
		except AssertionError:
			next_target_state = len(possible_target)
		except ValueError:
			next_target_state = len(possible_target)
		if ( (next_target_state == len(possible_target))):
			next_target_state = last_next_state
		elif next_target_state != last_next_state:
			pos_state.append(grid.state2vicon(controller.move(len(possible_target))[quad_name]))
		print next_target_state
		for elem in range(precompute_path_step):
			next_moves = controller.move(next_target_state)
			pos_state.append(grid.state2vicon(next_moves[quad_name]))
		res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol,corr=occupancy_grid.info.resolution/4)
		
		# if last_quad_list !=  quadController.is_current_list_1():
		# 	last_next_state = next_target_state
		# 	try:
		# 		#next_target_state = possible_target.index(grid.vicon2state(last_clicked_point))
		# 		next_target_state = closest_point(last_clicked_point , possible_target)
		# 		print "No problem = ", next_target_state
		# 	except AssertionError:
		# 		next_target_state = len(possible_target)
		# 	except ValueError:
		# 		next_target_state = len(possible_target)
		# 	send_waypoint(res_traj , path_pub)
		# 	if ( (next_target_state == len(possible_target))):
		# 		next_target_state = last_next_state
		# 	elif next_target_state != last_next_state:
		# 		pos_state.append(grid.state2vicon(controller.move(len(possible_target))[quad_name]))
		# 	print next_target_state
		# 	for elem in range(precompute_path_step):
		# 		next_moves = controller.move(next_target_state)
		# 		pos_state.append(grid.state2vicon(next_moves[quad_name]))
		# 	res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol,corr=occupancy_grid.info.resolution/2)
		# 	print "current list changed !!!"
		# 	if quadController.is_current_list_1():
		# 		quadController.current_path_2 = res_traj.pva
		# 		#send_waypoint(res_traj , path_pub,ind_seq=1)
		# 	else:
		# 		quadController.current_path_1 = res_traj.pva
		# 		#send_waypoint(res_traj , path_pub,ind_seq=0)
		# 	last_quad_list = quadController.is_current_list_1()
		#occupancy_wait.sleep()

		#res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol,corr=occupancy_grid.info.resolution/2)
		send_waypoint(res_traj , path_pub)

		for elem in res_traj.pva:
			pva_callback(elem)
			pva_pub.publish(elem)
			wait_rate.sleep()