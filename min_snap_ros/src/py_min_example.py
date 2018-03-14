#!/usr/bin/env python

import sys
import rospy
from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':

	rospy.init_node('min_snap_client_py', anonymous=True)

	#Set of waypoints and time between each waypoints
	x = [0.0 , 1.0 , 1.0 , 0.0]
	y = [0.0 , 0.0, 2.0 , 2.0]
	t = [0.0 , 0.5 , 1.5 , 2]

	########################## Example code for min snap when no inequality constraints #######################################

	rospy.wait_for_service('/min_snap_trajectory')
	simple_path_plan = SimplePathPlanRequest()

	#x waypoints
	simple_path_plan.x = x
	#x initial and end speed and acceleration
	simple_path_plan.velx_init =[0.0 , 0.0]
	simple_path_plan.accx_init =[0.0 , 0.0]

	#y waypoints
	simple_path_plan.y = y
	simple_path_plan.vely_init =[0.0 , 0.0]
	simple_path_plan.accy_init =[0.0 , 0.0]

	#fill in the z waypoints if it is a 3D trajectory --> Let it empty if it is only x and y that matters. idk for x and z ot y and z
	#simple_path_plan.z = z;

	#If you have corridors constraints-> proceed like the following
	simple_path_plan.corridors =[0.0 , 0.05, 0.0]

	#Fill the time list
	simple_path_plan.t =t

	#frequency of the returned path
	simple_path_plan.freq = 100

	#Call to the service
	min_snap_traj= rospy.ServiceProxy('/min_snap_trajectory',SimplePathPlan)
	res = min_snap_traj(simple_path_plan)

	#Solution are save in the list traj as a PVA
	X_no_ineq = [ val.pos.x for val in res.traj.pva ]
	Y_no_ineq = [ val.pos.y for val in res.traj.pva ]


	################################### End example of code with no ineequality ##############################################



	##################################  Example of code with inequality constraints ########################################

	# rospy.wait_for_service('/min_snap_trajectory')
	# simple_path_plan = SimplePathPlanRequest()
	# #x_axis = [ ConstraintAxis(m, list()) for m in x]
	# simple_path_plan.x = x;

	# simple_path_plan.y = y;

	# #fill in the z waypoints if it is a 3D trajectory --> Let it empty if it is only x and y that matters. idk for x and z ot y and z
	# #simple_path_plan.z = z;

	# #path_plan.waypoints.y = [ConstraintAxis(m, list()) for m in y]
	# #If you have corridors constraints-> proceed like the following
	# #path_plan.corridors =[0.0 , 0.05, 0.0]
	# path_plan.waypoints.t =t

	# #frequency of the returned path
	# path_plan.freq = 100
	# min_snap_traj= rospy.ServiceProxy('/min_snap_trajectory',PathPlan)
	# res = min_snap_traj(path_plan);
	# #for i in range(len(res.traj.position)):
	# #	print ("x : ",res.traj.position[i].x ,"vel_x: " , res.traj.velocity[i].x, "acc_x: " ,res.traj.acceleration[i].x," freq: ",res.traj.wait_freq[i]) 

	# X_no_ineq = [ val.x for val in res.traj.position ]
	# Y_no_ineq = [ val.y for val in res.traj.position ]

	# #################################   End example of code with inequality constraints ####################################

	plt.plot(X_no_ineq ,Y_no_ineq)
	plt.show()

	# # path_plan.waypoints.t =[0.0 , 2]
	# # res = min_snap_traj(path_plan);
	# # X = [ val.x for val in res.traj.position ]
	# # Y = [ val.y for val in res.traj.position ]
	# # plt.plot(X,Y)
