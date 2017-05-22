#!/usr/bin/env python
import numpy as np
import rospy
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import time
from quad_control.msg import TrajArray
from quad_control.srv import Traj, TrajResponse
from quad_control.msg import trajData

def handle_path_plan(req):

    inputs = {'p_init' : req.init_pos, 'p_final' : req.final_pos, 'v_init' : req.init_vel, 'v_final' : req.final_vel, 'a_init' : req.init_acc, 'a_final' : req.final_acc, 't_final' : req.final_time, 'freq' : req.sampling_rate} 
    
    (traj, time) = path_plan(**inputs)
    pos = [x for x in traj[0].tolist()[0]]
    vel = [x for x in traj[1].tolist()[0]]
    acc = [x for x in traj[2].tolist()[0]]
    
    traj_list = []

    for i in range(0, len(pos)):
        traj_point = trajData()

        traj_point.time = time[i]
        traj_point.position = pos[i]
        traj_point.velocity = vel[i]
        traj_point.acceleration = acc[i]
        traj_list.append(traj_point)

    trajectory = TrajResponse(traj_list)
    
    return trajectory


def path_plan(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
    t_init = 0
    (time, step) = np.linspace(t_init, t_final, num = (t_final - t_init)/(1/float(freq)),endpoint=True, retstep = True)
    A = np.matrix([[1,  t_init,  t_init**2,    t_init**3,    t_init**4,     t_init**5],\
                   [0,       1,   2*t_init,  3*t_init**2,  4*t_init**3,   5*(t_init**4)],\
                   [0,       0,          2,     6*t_init, 12*t_init**2,  20*(t_init**3)],\
                   [1, t_final, t_final**2,   t_final**3,   t_final**4,    t_final**5],\
                   [0,       1,  2*t_final, 3*t_final**2, 4*t_final**3,  5*(t_final**4)],\
                   [0,       0,          2,    6*t_final, 12*t_final**2, 20*t_final**3]])

    b = np.linalg.inv(A) * np.transpose(np.matrix([p_init, v_init, a_init, p_final, v_final, a_final]))


    ref1 = np.matrix(b[0] + b[1]*time + b[2]*time**2 + b[3]*time**3 + b[4]*time**4 + b[5]*time**5)
    ref2 = np.matrix(b[1] + 2*b[2]*time + 3*b[3]*time**2 + 4*b[4]*time**3 + 5*b[5]*time**4)
    ref3 = np.matrix(2*b[2] + 6*b[3]*time + 12*b[4]*time**2 + 20*b[5]*time**3)

    ref_inter =  np.concatenate((ref1,ref2))
    traj = np.concatenate((ref_inter,ref3))
    return (traj,time)

def test():

    x_inputs = {'p_init' : -4, 'p_final' : 18, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : 50}
    y_inputs = {'p_init' : -9, 'p_final' : 11, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : 10}
    z_inputs = {'p_init' : 0, 'p_final' : 1, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : 10}

    (RefX,time) = path_plan(**x_inputs);
    (RefY,time) = path_plan(**y_inputs);
    (RefZ,time) = path_plan(**z_inputs);

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-30,30)
    ax.set_ylim(-30,30)
    plt.ion()
    plt.show()

    traj = [x for x in zip(RefX[0].tolist()[0], RefY[0].tolist()[0])]

    codes = [Path.MOVETO]
    codes = codes + [Path.LINETO]*(len(traj)-1)     

    path = Path(traj, codes)
    patch = patches.PathPatch(path, facecolor='white')
    ax.add_patch(patch)
    plt.draw()
    plt.pause(2.05)

def path_plan_server():

    rospy.init_node('path_planner')
    s = rospy.Service('path_planner', Traj, handle_path_plan)
    print "Ready to plan your path."
    rospy.spin()

if __name__ == '__main__':
    path_plan_server()
    #test()