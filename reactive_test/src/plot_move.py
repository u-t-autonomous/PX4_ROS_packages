# from geometry_msgs.msg import TransformStamped
# from qcontrol_defs.msg import AttPVA
import sys
import rospy
from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import NullFormatter
import numpy as np
#from matplotlib import style

# quad9_pos = Point()
# quad10_pos = Point()
# quad9_sp = Point()
# quad10_sp = Point()

# def save_quad9_vicon(msg):
# 	global quad9_pos
#   	quad9_pos.x = msg->transform.translation.x
#   	quad9_pos.y = msg->transform.translation.y

# def save_quad10_vicon(msg):
# 	global quad10_pos
#   	quad10_pos.x = msg->transform.translation.x
#   	quad10_pos.y = msg->transform.translation.y

# def save_quad9_sp(msg):
# 	global quad9_sp
#   	quad9_sp.x = msg.posX_roll
#   	quad9_sp.y = msg.posY_pitch

# def save_quad10_sp(msg):
# 	global quad10_sp
#   	quad10_sp.x = msg.posX_roll
#   	quad10_sp.y = msg.posY_pitch
Y_MINIMUM = -5.5 #1.5
X_MINIMUM = -5.5

Y_MAXIMUM = 5.5
X_MAXIMUM = 5.5

X_NUMBER_TILE = 100
Y_NUMBER_TILE = 100

xstep = (X_MAXIMUM- X_MINIMUM)/X_NUMBER_TILE
ystep = (Y_MAXIMUM- Y_MINIMUM)/Y_NUMBER_TILE
#style.use('fivethirtyeight')

#plt.rcParams['toolbar'] = 'None'
fig = plt.figure()
#ax1 = fig.gca()
ax1 = fig.add_subplot(1,1,1)
ax1.grid(True)
ax1.set_autoscale_on(False)
ax1.set_xticks(np.arange(X_MINIMUM, X_MAXIMUM+xstep,xstep))
ax1.set_yticks(np.arange(Y_MINIMUM, Y_MAXIMUM+xstep,xstep))

x = [-2.0 , -2.0 , 2.0 , 2.0,-1.3,0.2,1.34,0.5,-0.4,0.15,0.0]
y = [2.0 , -2.0, -2.0 , 2.0 ,1.3,-2.25,-1.0,1.5,0.32,-1.0,0.0]
t = [0.0 , 0.5 , 1.5 , 2 ,3 , 4 , 5 , 5.8 ,6.8 , 7.8 , 8.4]

rospy.init_node('plotter', anonymous=True)
rospy.wait_for_service('/min_snap_trajectory')
path_plan = PathPlanRequest()
x_axis = [ ConstraintAxis(m, list()) for m in x]
path_plan.waypoints.x = x_axis
path_plan.waypoints.y = [ConstraintAxis(m, list()) for m in y]
path_plan.waypoints.corridors =[0.0 , 0.0, 0.0 , 2,1.0 ,0.0,0.0,0.0,0,0]
path_plan.waypoints.t =t
path_plan.freq = 100
min_snap_traj= rospy.ServiceProxy('/min_snap_trajectory',PathPlan)
last_time = rospy.Time.now()
res = min_snap_traj(path_plan);
print ("duration = " , (rospy.Time.now() - last_time).to_sec())
#for i in range(len(res.traj.position)):
#	print ("x : ",res.traj.position[i].x ,"vel_x: " , res.traj.velocity[i].x, "acc_x: " ,res.traj.acceleration[i].x," freq: ",res.traj.wait_freq[i]) 

t_vect = list()
t_vect.append(t[0])
for p in range(len(res.traj.pva)-1):
	t_vect.append(t_vect[p] + (1.0/res.traj.wait_freq[p]))
Y = [ val.pos.x for val in res.traj.pva ]
X = [ -val.pos.y for val in res.traj.pva ]

def animate(i):
	graph_data = open('data_plot.txt','r').read()
	lines = graph_data.split('\n')
	q9_pos_x =[]
	q9_pos_y =[]
	q10_pos_x =[]
	q10_pos_y =[]
	q9_sp_x =[]
	q9_sp_y =[]
	q10_sp_x =[]
	q10_sp_y =[]
	for line in lines :
		if(len(line) > 1):
			curr_line = line.split(',')
			if (len(curr_line) != 8):
				continue
			x1,y1,x2,y2,x3,y3,x4,y4 = curr_line
			q9_pos_x.append(float(x1))
			q9_pos_y.append(-float(y1))
			q10_pos_x.append(float(x2))
			q10_pos_y.append(-float(y2))
			q9_sp_x.append(float(x3))
			q9_sp_y.append(-float(y3))
			q10_sp_x.append(float(x4))
			q10_sp_y.append(-float(y4))
	ax1.clear()
	ax1.grid(True)
	ax1.patch.set_facecolor((0.9059,1.0,1.0))
	ax1.set_autoscale_on(False)
	ax1.set_xticks(np.arange(X_MINIMUM, X_MAXIMUM+xstep,xstep))
	ax1.set_yticks(np.arange(Y_MINIMUM, Y_MAXIMUM+xstep,xstep))
	#ax1.plot([X_MINIMUM,X_MINIMUM,X_MAXIMUM,X_MAXIMUM],[Y_MINIMUM,Y_MAXIMUM,Y_MAXIMUM,Y_MINIMUM])
	ax1.plot(q9_pos_y,q9_pos_x,'-r',label='sys_vicon',linewidth=1.0)
	ax1.plot(q10_pos_y,q10_pos_x,'-g',label='env_vicon',linewidth=1.0)
	ax1.plot(X,Y,'-k',label='min_snap_ref',linewidth=1.0)
	#ax1.plot(q9_sp_y,q9_sp_x,'-b',label='sys_setpoint',linewidth=3.0)
	#ax1.plot(q10_sp_y,q10_sp_x,'-m',label='env_setpoint',linewidth=3.0)
	ax1.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
	#ax1.legend()

ani = animation.FuncAnimation(fig,animate,interval=1000)

plt.rc('grid', linestyle="-", color='#D3D3D3')
plt.grid(True)
plt.show()
