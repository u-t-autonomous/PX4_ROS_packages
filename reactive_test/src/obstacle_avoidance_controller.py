#!/usr/bin/env python
import sys
import rospy
import re
from quad_control.srv import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from qcontrol_defs.msg import AttPVA
from qcontrol_defs.srv import *
from time import sleep
import numpy as np
from time import sleep
from math import floor

#from ObstacleController import ObstacleCtrl
from followMeController10_2_2 import FollowCtrl10_2_2

from visualization_msgs.msg import Marker
from threading import Thread,RLock

X_NUMBER_TILE = 10
Y_NUMBER_TILE = 10

Y_MINIMUM = -5 #1.5
X_MINIMUM = -5

Y_MAXIMUM = 5
X_MAXIMUM = 5

Z_LEVEL = 1.4
SYS_PERIMETER = 2


ENV_NAME = "/Quad10"
SYS_NAME = "/Quad9"

curr_color = [1.0,0.0,0.0]
ENV_OBSV_RATE = 100
SYS_MAX_TIME_STEP = rospy.Duration(8.0) #In second

def send_setpoint(init_point,end_point):
    marker = Marker()
    marker.header.frame_id = "1"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "setpoint_target"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.color.r = curr_color[0]
    marker.color.g = curr_color[1]
    marker.color.b = curr_color[2]
    marker.color.a = 1.0
    #marker.scale.x = grid.blocklengthX-0.05
    #marker.scale.y = grid.blocklengthY-0.05
    marker.scale.z = 0.05
    marker.points.append(init_point)
    marker.points.append(end_point)
    set_point_pub.publish(marker)

def takeoff(quad_name):
    rospy.wait_for_service(quad_name+'/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
    cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

def start_position_control(quad_name):
    rospy.wait_for_service(quad_name+'/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
    cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
    cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

def land(quad_name):
    rospy.wait_for_service(quad_name+'/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.start_landing = CommandActionRequest.START_LANDING_TRUE;
    cmd_srv = rospy.ServiceProxy(quad_name+'/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

def gen_speed_from_pos(init_pos,final_pos,delta_t):
	return Point((final_pos.x-init_pos.x)/delta_t,(final_pos.y-init_pos.y)/delta_t,(final_pos.z-init_pos.z)/delta_t)

class Sys_node(Thread):
    def __init__(self,sys_name,position_pub=None):
        Thread.__init__(self)
        self.sys_name = sys_name
        self.position = Point()
        self.lock = RLock()
        self.next_position = Point()
        self.next_duration = SYS_MAX_TIME_STEP.to_sec()
        self.publisher = position_pub

    def handle_position(self, msg):
        self.lock.acquire()
        self.position.x = msg.transform.translation.x
        self.position.y = msg.transform.translation.y
        self.position.z = Z_LEVEL
        self.lock.release()

    def set_next_target(self, duree ,next_position):
        self.lock.acquire()
        self.next_position = next_position
        self.next_duration = duree
        self.lock.release()

    def init_position(self,init_pos):
    	r = rospy.Rate(1)
        rospy.wait_for_service(self.sys_name+'/qcontrol/commands')
        cmdAction = CommandActionRequest()
        cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
        cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
        cmd_srv = rospy.ServiceProxy(self.sys_name+'/qcontrol/commands', CommandAction)
        cmd_srv(cmdAction)
        r.sleep()
        self.lock.acquire()
        pva_msg = get_att_pva(init_pos,gen_speed_from_pos(self.position,init_pos,10))
        self.next_position = init_pos
        self.lock.release()
        self.publisher.publish(pva_msg)
        r.sleep()

    def get_relative_real_state(self,grid):
    	self.lock.acquire();
    	exp_state = grid.vicon2state(self.next_position)
    	real_state = grid.vicon2state(self.position)
    	exp_state_row = exp_state/X_NUMBER_TILE
    	exp_state_col = exp_state%Y_NUMBER_TILE
    	real_state_row = real_state/X_NUMBER_TILE
    	real_state_col = real_state%Y_NUMBER_TILE
    	rel_state_col = exp_state_col- real_state_col
    	rel_state_row = exp_state_row- real_state_row
    	rel_state_row = rel_state_row + SYS_PERIMETER
    	rel_state_col =  rel_state_col + SYS_PERIMETER
    	self.lock.release();
    	return (rel_state_row,rel_state_col)

    def run(self):
    	sys_delay = rospy.Rate(50)
    	while not rospy.is_shutdown():
    		self.lock.acquire()
    		next_move = self.next_position
    		speed_to_target = gen_speed_from_pos(self.position,self.next_position,self.next_duration)
    		self.lock.release()
    		pva_msg = get_att_pva(next_move,speed_to_target)
    		self.publisher.publish(pva_msg)
    		sys_delay.sleep()


class Quad:

    def __init__(self,name):
        self.quad_name = name
        self.current_pos = Point()
        self.start_simu = False

    def handle_position(self,msg):
        self.current_pos.x = msg.transform.translation.x
        self.current_pos.y = msg.transform.translation.y
        self.current_pos.z = Z_LEVEL

    def grid_state(self,curr_grid):
        state = curr_grid.vicon2state(self.current_pos)
        return state

    def get_current_pos(self):
        res = self.current_pos
        return res

class Grid:

    def __init__(self, x, y, z, base, maximum):

        self.x = x
        self.y = y
        self.base = base
        self.maximum = maximum
        self.blocklengthX = (float(maximum.x - base.x))/x
        self.blocklengthY = (float(maximum.y - base.y))/y
        self.drone_pos  = Point()

    def vicon2state(self, position , sigErr = False):
        new_position = Point()
        position.x = max(position.x,self.base.x)
        position.x = min(position.x,self.maximum.x)
        position.y = max(position.y,self.base.y)
        position.y = min(position.y,self.maximum.y)
        #assert(position.x >= self.base.x and position.x <= self.maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, base.x, maximum.x)
        #assert(position.y >= self.base.y and position.y <= self.maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, base.y, maximum.y)
        new_position.y = int((position.y - self.base.y)/self.blocklengthY)
        new_position.x = int((position.x - self.base.x)/self.blocklengthX)
        assert(new_position.x >= 0 and new_position.x <= self.x), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.x, 0, self.x)
        assert(new_position.y >= 0 and new_position.y <= self.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.y, 0, self.y)
        state = floor(new_position.y) + (floor(new_position.x)*self.x)
        return int(state)

    def state2vicon(self, state):
        xaxis = state / self.x
        yaxis = state % self.x
        position = Point()
        position.x = self.base.x + (self.blocklengthX/2.) + (xaxis * self.blocklengthX )
        position.y = self.base.y + (self.blocklengthY/2.) + (yaxis * self.blocklengthY )
        position.z = self.base.z
        return position

def get_att_pva(current_target_pos,current_target_vel):
    att_msg = AttPVA()
    att_msg.use_position = True
    att_msg.use_speed = True
    att_msg.use_acceleration = False
    att_msg.use_rate = False
    att_msg.use_yaw = True

    att_msg.yaw = -0.0
    att_msg.posZ_thrust =current_target_pos.z
    att_msg.posY_pitch = current_target_pos.y
    att_msg.posX_roll = current_target_pos.x

    att_msg.velZ=current_target_vel.z
    att_msg.velY=current_target_vel.y
    att_msg.velX=current_target_vel.x

    return att_msg

if __name__ == "__main__":
    current_controller = FollowCtrl10_2_2()
    #Grid definition
    base = Point()
    base.x = X_MINIMUM
    base.y = Y_MINIMUM
    base.z = Z_LEVEL
    maximum = Point()
    maximum.x = X_MAXIMUM
    maximum.y = Y_MAXIMUM
    maximum.z = Z_LEVEL

    grid = Grid(X_NUMBER_TILE, Y_NUMBER_TILE,Z_LEVEL, base, maximum)
    quad_env1 = Quad("env1")
    sys_thread = Sys_node(SYS_NAME)

    rospy.init_node('system_node', anonymous=True)
    rospy.Subscriber("/vicon"+SYS_NAME+SYS_NAME, TransformStamped, sys_thread.handle_position)
    rospy.Subscriber("/vicon"+ENV_NAME+ENV_NAME, TransformStamped, quad_env1.handle_position)

    sys_traj_pub = rospy.Publisher(SYS_NAME+"/qcontrol/att_pva", AttPVA, queue_size=10)
    set_point_pub = rospy.Publisher("visualization_marker",Marker,queue_size=1)

    sys_thread.publisher = sys_traj_pub

    wait_rate = rospy.Rate(ENV_OBSV_RATE)

    raw_input('Lock the environment in the grid ... ')
    previous_env_state = quad_env1.grid_state(grid)
    next_position = grid.state2vicon(current_controller.move(SYS_PERIMETER,previous_env_state,SYS_PERIMETER)['loc'])
    sys_thread.init_position(next_position)
    raw_input('start playing against sys ...')
    sys_thread.start()
    last_state_mod = rospy.Time.now()
    #Finally start the turned based routine
    while not rospy.is_shutdown():
        curr_env_state = quad_env1.grid_state(grid)
        if previous_env_state == curr_env_state :
            if(rospy.Time.now()-last_state_mod >= SYS_MAX_TIME_STEP):
            	(sys_delta_row,sys_delta_col) =  sys_thread.get_relative_real_state(grid)
                next_state = current_controller.move(sys_delta_row,curr_env_state,sys_delta_col)['loc']
                sys_thread.set_next_target(SYS_MAX_TIME_STEP.to_sec(),grid.state2vicon(next_state))
                last_state_mod = rospy.Time.now()
        else:
            delta_t = rospy.Time.now() - last_state_mod
            if delta_t.to_sec() == 0:
            	delta_t = rospy.Duration(0.1)
            (sys_delta_row,sys_delta_col) =  sys_thread.get_relative_real_state(grid)
            try:
            	next_state = current_controller.move(sys_delta_row,curr_env_state,sys_delta_col)['loc']
            except ValueError:
            	next_state = current_controller.move(SYS_PERIMETER,curr_env_state,SYS_PERIMETER)['loc']
            sys_thread.set_next_target(delta_t.to_sec()/2,grid.state2vicon(next_state))
            print('NEW STATE : ',curr_env_state,' -- OLD STATE : ',previous_env_state,' -- REAL SYS POS: ',sys_delta_row,' , ',sys_delta_col,' -- SYS STATE : ',next_state,' -- DELTA_T : ',delta_t.to_sec())
            previous_env_state = curr_env_state
            last_state_mod = rospy.Time.now()
        wait_rate.sleep()