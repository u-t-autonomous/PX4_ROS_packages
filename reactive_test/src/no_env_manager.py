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
from sys3_controller import NoEnvController

from visualization_msgs.msg import Marker

X_NUMBER_TILE = 5
Y_NUMBER_TILE = 5

Y_MINIMUM = -5 #1.5
X_MINIMUM = -5

Y_MAXIMUM = 5
X_MAXIMUM = 5

Z_LEVEL = 1.4


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

def send_trajectory(traj_3d, sampling_rate,publisher):
    r = rospy.Rate(sampling_rate)
    for i in range(len(traj_3d.time)):
        att_msg = get_att_pva(traj_3d.position[i],traj_3d.velocity[i],traj_3d.acceleration[i])
        publisher.publish(att_msg)
        r.sleep()

def get_traj(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
    rospy.wait_for_service('path_planner')
    try:
        traj_srv = rospy.ServiceProxy('path_planner', Traj)
        resp1 = traj_srv(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq)
        return resp1.trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def make_3d_traj(x, y, z):
    trajlist = TrajArray()
    for i in range(0,len(x)):
        twist = Twist()
        wrench = Wrench()
        pose = Pose()
        twist.linear.x = x[i].velocity
        twist.linear.y = y[i].velocity
        twist.linear.z = z[i].velocity
        wrench.force.x = x[i].acceleration
        wrench.force.y = y[i].acceleration
        wrench.force.z = z[i].acceleration
        pose.position.x = x[i].position
        pose.position.y = y[i].position
        pose.position.z = z[i].position
        trajlist.time.append(x[i].time)
        trajlist.velocity.append(twist)
        trajlist.acceleration.append(wrench)
        trajlist.position.append(pose)
    return trajlist

def get_traj_3d(current_pos,next_pos,freq,duration):
    x_inputs = {'p_init' : current_pos.x, 'p_final' : next_pos.x, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
    y_inputs = {'p_init' : current_pos.y, 'p_final' : next_pos.y, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
    z_inputs = {'p_init' : current_pos.z, 'p_final' : next_pos.z, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : duration, 'freq' : freq}
    traj_x = get_traj(**x_inputs)
    traj_y = get_traj(**y_inputs)
    traj_z = get_traj(**z_inputs)
    return make_3d_traj(traj_x, traj_y, traj_z)

def move_quad(current_pos,next_pos,freq,duration,publisher):
    traj_3d = get_traj_3d(current_pos,next_pos,freq,duration)
    send_trajectory(traj_3d,freq,publisher)

if __name__ == "__main__":

    current_controller = FollowMeCtrl_50_obs()

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

    rospy.init_node('min_snap_tulip', anonymous=True)

    snap_traj = rospy.Publisher(SYS_NAME+"/no_env/snap", TransformStamped, queue_size=10)
    discrete_traj = rospy.Publisher(ENV_NAME+"/no_env/discrete", TransformStamped, queue_size=10)

    #vis_pub = rospy.Publisher("visualization_marker_array",MarkerArray,queue_size=1)
    #set_point_pub = rospy.Publisher("visualization_marker",Marker,queue_size=1)

    #Simulate tulip over a certain time
    max_iter = 20
    for i in range(max_iter):
        


    x = [-2.0 , -2.0 , 2.0 , 2.0,-1.3,0.2,1.34,0.5,-0.4,0.15,0.0]
    y = [2.0 , -2.0, -2.0 , 2.0 ,1.3,-2.25,-1.0,1.5,0.32,-1.0,0.0]
    t = [0.0 , 0.5 , 1.5 , 2 ,3 , 4 , 5 , 5.8 ,6.8 , 7.8 , 8.4]

    rospy.wait_for_service('/min_snap_trajectory')
    path_plan = PathPlanRequest()
    x_axis = [ ConstraintAxis(m, list()) for m in x]
    path_plan.waypoints.x = x_axis
    path_plan.waypoints.y = [ConstraintAxis(m, list()) for m in y]
    path_plan.waypoints.corridors =[0.0 , 0.0, 0.0 , 2,1.0 ,0.0,0.0,0.0,0,0]
    path_plan.waypoints.t =t
    path_plan.freq = 100
    min_snap_traj= rospy.ServiceProxy('/min_snap_trajectory',PathPlan)
    res = min_snap_traj(path_plan);
    target_point_ = []
    env_target_point = []
    for elem in res.traj.pva:
        elem.pos.z = Z_LEVEL
        print elem.pos.x
        target_point_.append(grid.vicon2state(elem.pos))
    for i in target_point_:
        if i not in env_target_point:
            env_target_point.append(i)
    #######################################################

    env_thread = Env_node(ENV_NAME,quad_env1,grid,env_target_point)
    sys_thread = Sys_node(SYS_NAME,env_thread,current_controller)


    rospy.Subscriber("/joy",Joy,env_thread.handle_joy)

    wait_rate = rospy.Rate(1)

    #Starting environment thread
    #env_thread.start()
    while not env_thread.is_posctl:
        wait_rate.sleep()

    #Send env to an initial position
    env_thread.env_quad_init(env_traj_pub)
    wait_rate.sleep()
    #send sys to his initial position while env is not moving
    sys_thread.sys_quad_init(sys_traj_pub)
    wait_rate.sleep()

    raw_input("Start ... ")
    wait_rate = rospy.Rate(SEND_POS_FREQ)
    #Finally start the turned based routine
    while not rospy.is_shutdown():
        env_thread.move(env_traj_pub)
        wait_rate.sleep()
        sys_thread.move(sys_traj_pub)
        wait_rate.sleep()