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

from math import floor

import add_target_controller as tulip_controller


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
		
def generate_traj(x , y , traj_time , corr=None , freq = 20):
	rospy.wait_for_service('/min_snap_trajectory')
	simple_path_plan = SimplePathPlanRequest()
	simple_path_plan.x = x
	simple_path_plan.y = y
	#simple_path_plan.z = z

	simple_path_plan.velx_init = [0.0 , 0.0]
	simple_path_plan.accx_init = [0.0 , 0.0]
	simple_path_plan.vely_init = [0.0 , 0.0]
	simple_path_plan.accy_init = [0.0 , 0.0]
	#simple_path_plan.velz_init = [0.0 , 0.0]
	#simple_path_plan.accz_init = [0.0 , 0.0]

	#Set corridors constraints if there exist any
	if not corr is None:
		for elem in corr:
			simple_path_plan.corridors.append(corr)

	#Set Time and frequency 
	simple_path_plan.t = traj_time
	simple_path_plan.freq = freq

	#connect to the server and send the request
	min_snap_traj = rospy.ServiceProxy('/min_snap_trajectory' , SimplePathPlan)
	res = min_snap_traj(simple_path_plan)
	#send the waypoint
	#send_waypoint(res.traj.pva , freq)
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
		pos_stamp = PoseStamped()
		pos_stamp.pose.position = elem.pos
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
	global next_target_selected
	is_static = True
	last_clicked_point = msg.pose.pose.position
	next_target_selected = True

def publish_user_setpoint(curr_position , m_color=[1.0,0.0,0.0] , cube_size=0.6):
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
	marker.color.a = 1.0
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
	quater = tf.transformations.quaternion_from_euler(0.0,0.0,msg.yaw)
	pos.pose.orientation.x = quater[0]
	pos.pose.orientation.y = quater[1]
	pos.pose.orientation.z = quater[2]
	pos.pose.orientation.w = quater[3]
	set_point_pub.publish(pos)
	msg_count = msg_count +1

if __name__ == "__main__":

	rospy.init_node('static_emvironment_node')

	#global occupancy_grid , last clicked point variable
	occupancy_grid = OccupancyGrid()
	last_clicked_point = Point()
	next_target_selected = False
	marker_id = 0
	msg_count =0

	#Get some subscribers to the occupancy grid and click point for human target set point
	occupancy_sub = rospy.Subscriber('/projected_map', OccupancyGrid , get_occupancy_grid)
	rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped ,  get_clicked_point)


	#USER setpoint target publisher
	click_point_publisher = rospy.Publisher('/user_target_point' , Marker , queue_size=10)
	set_point_pub = rospy.Publisher('Quad9/qcontrol/pva_setpoint', PoseStamped  , queue_size=10)

	#Start RVIZ and publish occupancy grid
	raw_input('Start RVIZ and LAUNCH OCCUPANCY GRID node (Octomap node with the registered map)')

	####### Initialize grid specifications #################

	Z_LEVEL = 0.0
	X_NUMBER_TILE = occupancy_grid.info.width # X correspond to the larger size -> width
	Y_NUMBER_TILE = occupancy_grid.info.height	# Y height correspond to the smaller -> height

	base = Point()
	base.x = occupancy_grid.info.origin.position.x
	base.y = occupancy_grid.info.origin.position.y
	base.z = Z_LEVEL

	maxi = Point()
	maxi.x = occupancy_grid.info.resolution * X_NUMBER_TILE + base.x
	maxi.y = occupancy_grid.info.resolution * Y_NUMBER_TILE + base.y
	maxi.z = Z_LEVEL

	grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)

	print grid.x , grid.y , grid.base , grid.maximum , grid.blocklengthX , grid.blocklengthY

	####### End grid specifications #############################

	#unsubscribe and subscribe to occupancy grid until the map is static
	static_obstacles = set()
	occupancy_wait = rospy.Rate(30)
	is_static = False
	while not is_static:
		occupancy_sub.unregister()
		occupancy_wait.sleep()
		occupancy_sub = rospy.Subscriber('/projected_map', OccupancyGrid , get_occupancy_grid)
		for elem in range(occupancy_grid.info.width * occupancy_grid.info.height):
			if occupancy_grid.data[elem] == 100:
				#print elem
				elem_x = elem % occupancy_grid.info.width
				elem_y = elem / occupancy_grid.info.width
				static_obstacles.add(elem_y + elem_x * occupancy_grid.info.height)
				publish_user_setpoint(grid.state2vicon(elem_y + elem_x * occupancy_grid.info.height))



	#Initialise PVA setpoint publisher for the different quad -- Also initialize all systems specifications
	publisher_dict = dict()
	sys_specs = dict()

	number_quad =  int( raw_input("Number of quads : "))

	for elem in range(number_quad):
		(quad_name , waypoints) = get_information_quad(elem)
		sys_specs[quad_name] = {'waypoints': grid.vicon2state_list(waypoints) , 'surface': []}
		publisher_dict[quad_name] = (rospy.Publisher('/' + quad_name + '/qcontrol/pva_control' , PVA , queue_size=10),rospy.Publisher('/' + quad_name + '/qcontrol/path_pva' , Path , queue_size=10))

	#Some takeoff procedure and going to the initial state
	number_possible_target = int(raw_input('Number of possible target : '))
	possible_target = list()
	for elem in range(number_possible_target):
		raw_input(' Click on a target cell/Point and then press enter !')
		possible_target.append(grid.vicon2state(last_clicked_point))
		publish_user_setpoint(grid.state2vicon(grid.vicon2state(last_clicked_point)) ,[0.0,1.0,0.0], occupancy_grid.info.resolution)

	#instanciate tulip controller and create the controller
	name = raw_input("Controller name : ")
	agent_controller = tulip_controller.AgentController(name , X_NUMBER_TILE , Y_NUMBER_TILE ,sys_specs ,possible_target,'Quad9', static_obstacles , 0 ,0)
	agent_controller.create_controller()

	next_target_selected = False
	freq_sol = 20
	precompute_path_step = 5
	precompute_path_duration = 5.0
	next_target_state = len(possible_target)
	controller = agent_controller.getController()
	wait_rate = rospy.Rate(freq_sol)
	raw_input('Now start the game :')
	while not rospy.is_shutdown():
		state_dict = dict()
		for elem in range(precompute_path_step):
			next_moves = controller.move(next_target_state)
			for sys_name, value in next_moves.items():
				if 'stage' in sys_name:
					continue
				if state_dict.get(sys_name) is None:
					state_dict[sys_name] = list()
				state_dict[sys_name].append(value)
		try:
			next_target_state = possible_target.index(grid.vicon2state(last_clicked_point))
		except AssertionError:
			next_target_state = len(possible_target)
		except IndexError:
			next_target_state = len(possible_target)
		print "current target = " , next_target_state
		final_solution = dict()
		size_solution = 0
		for sys_name , target_state in state_dict.items():
			pos_state = from_state_to_pva(target_state , grid)
			res_traj = generate_traj([elem.x for elem in pos_state ],[elem.y for elem in pos_state ],[(i*precompute_path_duration)/(len(pos_state)-1) for i in range(len(pos_state))],freq=freq_sol)
			send_waypoint(res_traj , publisher_dict[sys_name][1])
			final_solution[sys_name] = res_traj.pva
			size_solution = len(res_traj.pva)

		for elem in range(size_solution):
			for sys_name , pva_sys in final_solution.items():
				pva_callback(pva_sys[elem])
				publisher_dict[sys_name][0].publish(pva_sys[elem])
				wait_rate.sleep()
