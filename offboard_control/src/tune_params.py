#!/usr/bin/env python

import sys
import rospy
from qcontrol_defs.msg import *
from qcontrol_defs.srv import *
from geometry_msgs.msg import PoseStamped
import tf
from threading import Thread
import math

msg_count = 0

class Tuning(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self):
		rospy.wait_for_service('/Quad9/qcontrol/pos_control_param')
		rospy.wait_for_service('/Quad9/qcontrol/system_params')
		pid_params = rospy.ServiceProxy('Quad9/qcontrol/pos_control_param',updatePx4param)
		sys_params = rospy.ServiceProxy('Quad9/qcontrol/system_params',updatePx4param)
		while not rospy.is_shutdown():
			pid_sys = raw_input("pid or sys : ")
			params = updatePx4paramRequest()
			if pid_sys == 'pid':
				val = raw_input('[Kpx Kpy Kpz Kvx Kvy Kvz Kix Kiy Kiz maxInteg_x maxInteg_y maxInteg_z] :')
				for elem in val.split(';'):
					params.data.append(float(elem))
				response = pid_params(params)
			else:
				val = raw_input('[mass gz thrustRatio] : ')
				for elem in val.split(';'):
					params.data.append(float(elem))
				response = sys_params(params)

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

if __name__ == '__main__':

	rospy.init_node('params_tuning_node' , anonymous=True)

	rospy.Subscriber('/Quad9/qcontrol/pva_control' , PVA , pva_callback)
	set_point_pub = rospy.Publisher('/Quad9/qcontrol/pva_setpoint', PoseStamped , queue_size=10)
	
	tuning = Tuning()
	tuning.start()

	rospy.spin()

	tuning.join()
