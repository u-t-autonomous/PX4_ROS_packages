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

def get_occupancy_grid(msg):
	global occupancy_grid
	occupancy_grid = msg

def get_clicked_point(msg):
	global last_clicked_point
	global obstacle_selection
	if obstacle_selection:
		global static_obstacles
		global grid
		global resolution
		static_obstacles.add(grid.vicon2state(msg.pose.pose.position))
		publish_user_setpoint(msg.pose.pose.position , m_color=[0.0 , 0.0 , 0.0 ,1.0] , cube_size=resolution)
	else:
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
	marker.pose.position.z = Z_LEVEL
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
		publish_user_setpoint(grid.state2vicon(grid.vicon2state(last_clicked_point)) , m_color=[1.0 , 1.0 , 0.0 , 1.0] ,cube_size=resolution)
		print grid.vicon2state(last_clicked_point)
	return (quad_name , waypoints)

if __name__ == "__main__":

	rospy.init_node('register_map_details')

	#global occupancy_grid , last clicked point variable
	occupancy_grid = OccupancyGrid()
	obstacle_selection = False
	last_clicked_point = Point()
	marker_id = 0
	msg_count =0

	#Get some subscribers to the occupancy grid and click point for human target set point
	occupancy_sub = rospy.Subscriber('/projected_map', OccupancyGrid , get_occupancy_grid)
	rospy.Subscriber('/initialpose' , PoseWithCovarianceStamped ,  get_clicked_point)


	#USER setpoint target publisher
	click_point_publisher = rospy.Publisher('/user_target_point' , Marker , queue_size=10)

	#Start RVIZ and publish occupancy grid
	#raw_input('Start RVIZ and LAUNCH OCCUPANCY GRID node (Octomap node with the registered map)')

	####### Initialize grid specifications #################
	use_occupancy = int (raw_input('Use of occupancy grid : '))


	static_obstacles = set()

	Z_LEVEL = float(raw_input('Enter Z level : '))
	occupancy_wait = rospy.Rate(200)
	if use_occupancy == 1:
		X_NUMBER_TILE = occupancy_grid.info.width # X correspond to the larger size -> width
		Y_NUMBER_TILE = occupancy_grid.info.height	# Y height correspond to the smaller -> height
		resolution = occupancy_grid.info.resolution

		base = Point()
		base.x = occupancy_grid.info.origin.position.x
		base.y = occupancy_grid.info.origin.position.y
		base.z = Z_LEVEL

		maxi = Point()
		maxi.x = occupancy_grid.info.resolution * X_NUMBER_TILE + base.x
		maxi.y = occupancy_grid.info.resolution * Y_NUMBER_TILE + base.y
		maxi.z = Z_LEVEL
		grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)
		for elem in range(occupancy_grid.info.width * occupancy_grid.info.height):
			if occupancy_grid.data[elem] == 100:
				elem_x = elem % occupancy_grid.info.width
				elem_y = elem / occupancy_grid.info.width
				static_obstacles.add(elem_y + elem_x * occupancy_grid.info.height)
				publish_user_setpoint(grid.state2vicon(elem_y + elem_x * occupancy_grid.info.height), m_color=[0.0,0.0,0.0,1.0],cube_size=resolution)
				occupancy_wait.sleep()
	else:
		X_NUMBER_TILE = int (raw_input('Number of row : ')) # X correspond to the larger size -> width
		Y_NUMBER_TILE = int (raw_input('Number of col : ')) # Y height correspond to the smaller -> height
		resolution = float(raw_input('grid Cell size : '))

		raw_input('Click on the base point (0,0) state point --> It should be on the lower right for NED system')
		base = last_clicked_point
		base.z = Z_LEVEL

		maxi = Point()
		maxi.x = resolution * X_NUMBER_TILE + base.x
		maxi.y = resolution * Y_NUMBER_TILE + base.y
		maxi.z = Z_LEVEL

		grid = Grid(X_NUMBER_TILE ,Y_NUMBER_TILE , base , maxi)
		obstacle_selection = True
		raw_input("Click on static obstacles Then press enter to stop selecting obstacles")
		obstacle_selection = False
	####### End grid specifications #############################



	(quad_name , waypoints) = get_information_quad(0)
	waypoints_state = grid.vicon2state_list(waypoints)

	#Some takeoff procedure and going to the initial state

	number_possible_target = int(raw_input('Number of possible target : '))
	possible_target = list()
	#Add first point in possible target
	if len(waypoints) > 0:
		possible_target.append(waypoints_state[0])
	for elem in range(number_possible_target):
		raw_input(' Click on a target cell/Point and then press enter !')
		possible_target.append(grid.vicon2state(last_clicked_point))
		publish_user_setpoint(grid.state2vicon(grid.vicon2state(last_clicked_point)) ,[0.0,1.0,0.0,1.0], resolution)

	raw_input("Click on starting point : ")
	initial_state = grid.vicon2state(last_clicked_point)

	filename_output = raw_input('Filename output (No extension) : ')
	settings = open(filename_output+'.txt' , "w")

	settings.write('{row};{col};{resolution}\n'.format(row=X_NUMBER_TILE,col=Y_NUMBER_TILE,resolution=resolution))
	settings.write('{base_x};{base_y}\n'.format(base_x=base.x , base_y=base.y))
	settings.write(str(initial_state)+'\n')
	settings.write(";".join([str(state) for state in waypoints_state])+'\n')
	settings.write(";".join([str(state) for state in possible_target])+'\n')
	for row in range(X_NUMBER_TILE):
		settings.write(";".join( str(int(((X_NUMBER_TILE-1-row) * Y_NUMBER_TILE + (Y_NUMBER_TILE-1-col)) in static_obstacles))  for col in range(Y_NUMBER_TILE))+'\n')

