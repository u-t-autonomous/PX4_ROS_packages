#!/usr/bin/env python
import sys
import rospy

from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

#Client request to offboard server for arming motors and going into PVA control mode
def land(quad_name , takeoff_before= False):
	rospy.wait_for_service('/' + quad_name + '/qcontrol/commands')
	cmd_srv = rospy.ServiceProxy('/' + quad_name + '/qcontrol/commands', CommandAction)
	cmdAction = CommandActionRequest()
	cmdAction.start_landing = 1
	reponse = cmd_srv(cmdAction)

land('Quad9')
land('Quad10')
land('Quad8')