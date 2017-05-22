#!/usr/bin/env python
import sys
import rospy
from quad_control.srv import *
import time
import re
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from quad_control.msg import TrajArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from qcontrol_defs.msg import PVA
from qcontrol_defs.msg import PVAStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from qcontrol_defs.msg import AttPVA
from qcontrol_defs.msg import QuadState
from qcontrol_defs.srv import *
from time import sleep
import numpy as np
from time import sleep
from math import floor

takeoff_complete = False
currX =0
currY = 0
currZ = 0

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

def send_trajectory(traj_3d, sampling_rate):
    r = rospy.Rate(sampling_rate)
    for i in range(len(traj_3d.time)):
        att_msg = AttPVA()
        att_msg.use_position = True
        att_msg.use_speed = True
        att_msg.use_acceleration = False
        att_msg.use_rate = False
        att_msg.use_yaw = False
        att_msg.posZ_thrust =traj_3d.position[i].position.z
        att_msg.posY_pitch = traj_3d.position[i].position.y
        att_msg.posX_roll = traj_3d.position[i].position.x

        att_msg.velZ=traj_3d.velocity[i].linear.z
        att_msg.velY=traj_3d.velocity[i].linear.y
        att_msg.velX=traj_3d.velocity[i].linear.x

        att_msg.accZ=traj_3d.acceleration[i].force.z
        att_msg.accY=traj_3d.acceleration[i].force.y
        att_msg.accX=traj_3d.acceleration[i].force.x
        pub.publish(att_msg)
        #inter = (currX-att_msg.posX_roll)*(currX-att_msg.posX_roll) + (currY- att_msg.posY_pitch)*(currY- att_msg.posY_pitch) + (currZ - att_msg.posZ_thrust)*(currZ - att_msg.posZ_thrust)
        #while (inter > 0.25):
        #    inter = (currX-att_msg.posX_roll)*(currX-att_msg.posX_roll) + (currY- att_msg.posY_pitch)*(currY- att_msg.posY_pitch) + (currZ - att_msg.posZ_thrust)*(currZ - att_msg.posZ_thrust)
        #pva.t = rospy.Time.from_sec(traj_3d.time[i])
        #pva.pos = traj_3d.position[i]
        #pva.vel = traj_3d.velocity[i]
        r.sleep()

def quadStateSub(msg):
    global takeoff_complete
    takeoff_complete= msg.takeoff_complete
    #print("Wait for taking: ",is_takingoff," is landed: ",is_landed)

def get_traj(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
    rospy.wait_for_service('path_planner')
    try:
        traj_srv = rospy.ServiceProxy('path_planner', Traj)
        resp1 = traj_srv(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq)
        return resp1.trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def pos_info_sub(msg) :
    global currX
    global currY
    global currZ
    #currX = msg.transform.translation.x
    #currY = msg.transform.translation.y
    #currZ = msg.transform.translation.z
    currX = msg.pose.position.x
    currY = msg.pose.position.y
    currZ = msg.pose.position.z

if __name__ == "__main__":
    while_flag = 0    
    frequency = 5
    corner_time = 5 #in seconds

    rospy.init_node('quad_client', anonymous=True)
    #rospy.Subscriber("/qcontrol/att_pva", TransformStamped, grid.handle_position)
    rospy.Subscriber('/qcontrol/quad_info',QuadState,quadStateSub)
    #rospy.Subscriber('/vicon/Quad8/Quad8',TransformStamped,pos_info_sub)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,pos_info_sub)
    pub = rospy.Publisher('/qcontrol/att_pva', AttPVA, queue_size=10)

    x_init = -0.9
    y_init = 0.6
    z_init = 0.6
    cube_lengthX= 1.2
    cube_lengthY = 1.2
    cube_lengthZ = 1.2


    traj_x_1 = get_traj(x_init,x_init,0,0,0,0,corner_time,frequency)
    traj_y_1 = get_traj(y_init,y_init-cube_lengthY,0,0,0,0,corner_time,frequency)
    traj_z = get_traj(z_init,z_init,0,0,0,0,corner_time,frequency)
    traj_3d_1 = make_3d_traj(traj_x_1, traj_y_1, traj_z)

    traj_x_2 = get_traj(x_init,x_init+cube_lengthX,0,0,0,0,corner_time,frequency)
    traj_y_2 = get_traj(y_init-cube_lengthY,y_init-cube_lengthY,0,0,0,0,corner_time,frequency)
    #traj_z = get_traj(z_init,z_init,0,0,0,0,corner_time,frequency)
    traj_3d_2 = make_3d_traj(traj_x_2, traj_y_2, traj_z)

    traj_x_3 = get_traj(x_init+cube_lengthX,x_init+cube_lengthX,0,0,0,0,corner_time,frequency)
    traj_y_3 = get_traj(y_init-cube_lengthY,y_init,0,0,0,0,corner_time,frequency)
    #traj_z = get_traj(z_init,z_init,0,0,0,0,corner_time,frequency)
    traj_3d_3 = make_3d_traj(traj_x_3, traj_y_3, traj_z)

    traj_x_4 = get_traj(x_init+cube_lengthX,x_init,0,0,0,0,corner_time,frequency)
    traj_y_4 = get_traj(y_init,y_init,0,0,0,0,corner_time,frequency)
    #traj_z = get_traj(z_init,z_init,0,0,0,0,corner_time,frequency)
    traj_3d_4 = make_3d_traj(traj_x_4, traj_y_4, traj_z)

    #traj init

    traj_x_5 = get_traj(x_init,x_init,0,0,0,0,corner_time,frequency)
    traj_y_5 = get_traj(y_init,y_init,0,0,0,0,corner_time,frequency)
    traj_z_5 = get_traj(z_init,z_init+cube_lengthZ,0,0,0,0,corner_time,frequency)
    traj_z_6 = get_traj(z_init+cube_lengthZ,z_init,0,0,0,0,corner_time,frequency)
    traj_3d_5 = make_3d_traj(traj_x_5, traj_y_5, traj_z_5)

    traj_3d_up1 = make_3d_traj(traj_x_1,traj_y_2,traj_z_5)
    traj_3d_down1 = make_3d_traj(traj_x_1,traj_y_2,traj_z_6)
    traj_3d_up2 = make_3d_traj(traj_x_3,traj_y_2,traj_z_5)
    traj_3d_down2 = make_3d_traj(traj_x_3,traj_y_2,traj_z_6)
    traj_3d_up3 = make_3d_traj(traj_x_3,traj_y_4,traj_z_5)
    traj_3d_down3 = make_3d_traj(traj_x_3,traj_y_4,traj_z_6)

    #traj init
    traj_3d_init = make_3d_traj(traj_x_1,traj_y_5,traj_z)

    traj_z = get_traj(z_init+cube_lengthZ,z_init+cube_lengthZ,0,0,0,0,corner_time,frequency) 

    traj_3d_6 = make_3d_traj(traj_x_1, traj_y_1, traj_z)
    traj_3d_7 = make_3d_traj(traj_x_2, traj_y_2, traj_z)
    traj_3d_8 = make_3d_traj(traj_x_3, traj_y_3, traj_z)
    traj_3d_9 = make_3d_traj(traj_x_4, traj_y_4, traj_z)


    #traj_x_end = get_traj(x_init,x_init+cube_lengthX/2,0,0,0,0,corner_time,frequency)
    #traj_y_end = get_traj(y_init,y_init-cube_lengthY/2,0,0,0,0,corner_time,frequency)
    #traj_z_end = get_traj(z_init+cube_lengthZ,z_init+cube_lengthZ/2,0,0,0,0,corner_time,frequency)
    #traj_3d_end = make_3d_traj(traj_x_end,traj_y_end,traj_z_end)


    rospy.wait_for_service('/qcontrol/commands')

    cmdAction = CommandActionRequest()
    cmdAction.arm_motors = 1
    cmdAction.is_posctl = 1
    #cmdAction.arm_motors = 1
    cmd_srv = rospy.ServiceProxy('/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)
    #reponse = cmd_srv(cmdAction.is_poshld,cmdAction.is_posctl,cmdAction.attctl,cmdAction.arm_motors,cmdAction.start_takeoff,cmdAction.start_landing)

    wait_rate = rospy.Rate(20)
    wait_rate.sleep()
    wait_rate.sleep()

    count = 0;
    att_msg = AttPVA()
    att_msg.use_position = True
    att_msg.use_speed = False
    att_msg.use_acceleration = False
    att_msg.use_rate = False
    att_msg.use_yaw = False
    att_msg.posZ_thrust = z_init
    att_msg.posY_pitch = y_init
    att_msg.posX_roll = x_init

    while(count <15):
        count = count +1
        pub.publish(att_msg)
        wait_rate.sleep()
    
    print("Finish publishing start point ")
    bof = raw_input('Enter your input:')

    # cmdAction = CommandActionRequest()
    # cmdAction.is_posctl = 1
    # reponse = cmd_srv(cmdAction)
    # wait_rate.sleep()
    # wait_rate.sleep()
    # wait_rate.sleep()
    # wait_rate.sleep()
    # wait_rate.sleep()
    # wait_rate.sleep()

    print("starting trajectory !!!")
    #send_trajectory(traj_3d_init,frequency)
    send_trajectory(traj_3d_1,frequency)
    send_trajectory(traj_3d_up1,frequency)
    send_trajectory(traj_3d_down1,frequency)

    send_trajectory(traj_3d_2,frequency)
    send_trajectory(traj_3d_up2,frequency)
    send_trajectory(traj_3d_down2,frequency)

    send_trajectory(traj_3d_3,frequency)
    send_trajectory(traj_3d_up3,frequency)
    send_trajectory(traj_3d_down3,frequency)

    send_trajectory(traj_3d_4,frequency)

    send_trajectory(traj_3d_5,frequency)

    send_trajectory(traj_3d_6,frequency)
    send_trajectory(traj_3d_7,frequency)
    send_trajectory(traj_3d_8,frequency)
    send_trajectory(traj_3d_9,frequency)
    #send_trajectory(traj_3d_end,frequency)

    wait_rate.sleep()
    cmdAction = CommandActionRequest()
    cmdAction.start_landing = 1;
    cmd_srv = rospy.ServiceProxy('/qcontrol/commands', CommandAction)
    #reponse = cmd_srv(cmdAction.is_poshld,cmdAction.is_posctl,cmdAction.attctl,cmdAction.arm_motors,cmdAction.start_takeoff,cmdAction.start_landing)
    wait_rate.sleep()
    wait_rate.sleep()
    wait_rate.sleep()
    wait_rate.sleep()
    wait_rate.sleep()
    wait_rate.sleep()
    reponse = cmd_srv(cmdAction)
    print("Landing command launch !!!!")

    rospy.spin()
