#!/usr/bin/env python
import sys
import rospy

from qcontrol_defs.msg import *
from qcontrol_defs.srv import *

from geometry_msgs.msg import Point

from math import floor


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

# From a list of 2D x target waypoint and y target waypoint  generate a trajectory from
### min_snap algorithm to reach all these targets
# traj_time : can be a list of time to reach each waypoint target or just [0 t_end] the total duration
## of the trajectory 
# corr : specify some corridors constraints between intermediate waypoints
# freq : The number of intermediate point in each waypoint to waypoint path    
def generate_traj_2d(x , y , traj_time , corr=None , freq = 20):
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
    return res.traj

# From a list of 3D  : x,y and z targets waypoints,  generate a trajectory from
### min_snap algorithm to reach all these targets
# traj_time : can be a list of time to reach each waypoint target or just [0 t_end] the total duration
## of the trajectory 
# corr : specify some corridors constraints between intermediate waypoints
# freq : The number of intermediate point in each waypoint to waypoint path    
def generate_traj_3d(x , y , z , traj_time , corr=None , freq = 20):
    rospy.wait_for_service('/min_snap_trajectory')
    simple_path_plan = SimplePathPlanRequest()
    simple_path_plan.x = x
    simple_path_plan.y = y
    simple_path_plan.z = z

    simple_path_plan.velx_init = [0.0 , 0.0]
    simple_path_plan.accx_init = [0.0 , 0.0]
    simple_path_plan.vely_init = [0.0 , 0.0]
    simple_path_plan.accy_init = [0.0 , 0.0]
    simple_path_plan.velz_init = [0.0 , 0.0]
    simple_path_plan.accz_init = [0.0 , 0.0]

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
    return res.traj

#Client request to offboard server for arming motors and going into PVA control mode
def start_pva_control(quad_name , takeoff_before= False):
    rospy.wait_for_service(quad_name + '/qcontrol/commands')
    cmd_srv = rospy.ServiceProxy(quad_name + '/qcontrol/commands', CommandAction)
    takeoff_wait = rospy.Rate(0.2)  # We wait for 5 second before going in PVA control mode in case takeoff is asked
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

#Client request to offboard server for arming motors and going into POS control mode
def start_pos_control(quad_name , takeoff_before= False):
    rospy.wait_for_service(quad_name + '/qcontrol/commands')
    cmd_srv = rospy.ServiceProxy(quad_name + '/qcontrol/commands', CommandAction)
    takeoff_wait = rospy.Rate(0.2)  # We wait for 5 second before going in PVA control mode in case takeoff is asked
    if takeoff_before :
        cmdAction = CommandActionRequest()
        cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
        cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
        reponse =  cmd_srv(cmdAction)
        takeoff_wait.sleep()
    cmdAction = CommandActionRequest()
    cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
    cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
    reponse = cmd_srv(cmdAction)
    takeoff_wait.sleep()

# Arm motors and start taking off
def start_takeoff(quad_name):
    rospy.wait_for_service(quad_name + '/qcontrol/commands')
    cmd_srv = rospy.ServiceProxy(quad_name + '/qcontrol/commands', CommandAction)
    takeoff_wait = rospy.Rate(0.2)  # We wait for 5 second before going in PVA control mode in case takeoff is asked
    cmdAction = CommandActionRequest()
    cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
    cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
    reponse =  cmd_srv(cmdAction)
    takeoff_wait.sleep()

# Land the vehicule
def start_landing(quad_name):
    rospy.wait_for_service(quad_name + '/qcontrol/commands')
    cmd_srv = rospy.ServiceProxy(quad_name + '/qcontrol/commands', CommandAction)
    landing_wait = rospy.Rate(0.2)  # We wait for 5 second before going in any other mode
    cmdAction = CommandActionRequest()
    cmdAction.start_landing = CommandActionRequest.START_LANDING_TRUE
    reponse =  cmd_srv(cmdAction)
    landing_wait.sleep()