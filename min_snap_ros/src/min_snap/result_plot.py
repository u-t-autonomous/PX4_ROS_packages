import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax1 = fig.gca(projection='3d')

def animate(i):
	graph_data = open('results.txt','r').read()
	lines = graph_data.split('\n')
	x_size = []
	y_size = []
	z_size = []
	for line in lines :
		if(len(line) > 1):
			curr_line = line.split(',')
			if (len(curr_line) != 3):
				continue
			x,y,z = curr_line
			x_size.append(float(x))
			y_size.append(float(y))
			z_size.append(float(z))
	ax1.clear()
	ax1.grid(True)
	ax1.patch.set_facecolor((0.9059,1.0,1.0))
	#ax1.set_autoscale_on(False)
	#ax1.set_xticks(np.arange(X_MINIMUM, X_MAXIMUM+xstep,xstep))
	#ax1.set_yticks(np.arange(Y_MINIMUM, Y_MAXIMUM+xstep,xstep))
	#ax1.plot([X_MINIMUM,X_MINIMUM,X_MAXIMUM,X_MAXIMUM],[Y_MINIMUM,Y_MAXIMUM,Y_MAXIMUM,Y_MINIMUM])
	ax1.plot(x_size,y_size,'-r',linewidth=1.0)
	#ax1.plot(q10_pos_y,q10_pos_x,'-g',label='env_vicon',linewidth=1.0)
	#ax1.plot(X,Y,'-k',label='min_snap_ref',linewidth=1.0)
	#ax1.plot(q9_sp_y,q9_sp_x,'-b',label='sys_setpoint',linewidth=3.0)
	#ax1.plot(q10_sp_y,q10_sp_x,'-m',label='env_setpoint',linewidth=3.0)
	# ax1.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
 #           ncol=2, mode="expand", borderaxespad=0.)
	#ax1.legend()

ani = animation.FuncAnimation(fig,animate,interval=1000)

plt.rc('grid', linestyle="-", color='#D3D3D3')
#plt.ylabel('y value')
#plt.xlabel('x value')
#plt.zlabel('z value')
plt.grid(True)
plt.show()