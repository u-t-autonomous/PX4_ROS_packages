#!/usr/bin/env python
import sys
import rospy
from min_snap.msg import *
from min_snap.srv import *

import numpy as np
import matplotlib.pyplot as plt

x = [0.0 , 1.0 , 1.0 , 0.0]
y = [0.0 , 0.0, 2.0 , 2.0]

t = [0.0 , 0.5 , 1.5 , 2]

if __name__ == '__main__':
	rospy.init_node('test_min_snap_node', anonymous=True)
	rospy.wait_for_service('/min_snap_trajectory')
	path_plan = PathPlanRequest()
	x_axis = [ ConstraintAxis(m, list()) for m in x]
	path_plan.waypoints.x = x_axis
	path_plan.waypoints.y = [ConstraintAxis(m, list()) for m in y]
	path_plan.waypoints.corridors =[0.0 , 0.05, 0.0]
	path_plan.waypoints.t =t
	path_plan.freq = 100
	min_snap_traj= rospy.ServiceProxy('/min_snap_trajectory',PathPlan)
	res = min_snap_traj(path_plan);
	#for i in range(len(res.traj.position)):
	#	print ("x : ",res.traj.position[i].x ,"vel_x: " , res.traj.velocity[i].x, "acc_x: " ,res.traj.acceleration[i].x," freq: ",res.traj.wait_freq[i]) 

	t_vect = list()
	t_vect.append(t[0])
	for p in range(len(res.traj.position)-1):
		t_vect.append(t_vect[p] + (1.0/res.traj.wait_freq[p]))
	X = [ val.x for val in res.traj.position ]
	Y = [ val.y for val in res.traj.position ]
	plt.plot(X,Y)
	# path_plan.waypoints.t =[0.0 , 2]
	# res = min_snap_traj(path_plan);
	# X = [ val.x for val in res.traj.position ]
	# Y = [ val.y for val in res.traj.position ]
	# plt.plot(X,Y)
	plt.show()