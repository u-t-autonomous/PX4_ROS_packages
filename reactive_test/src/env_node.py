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

from sensor_msgs.msg import Joy

ENV_NAME = "/Quad10"
SYS_NAME = "/Quad9"

X_NUMBER_TILE = 8
Y_NUMBER_TILE = 8

Y_MINIMUM = -2.7
X_MINIMUM = -1.5

Y_MAXIMUM = 2.7
X_MAXIMUM = 1.5

Z_LEVEL = 1.4

ONE_MOVE_DURATION = 7
SEND_POS_FREQ = 10

quad_current_pos = Point()

base  = Point(X_MINIMUM,Y_MINIMUM,Z_LEVEL)
maximum = Point(X_MAXIMUM,Y_MAXIMUM,Z_LEVEL)
blocklengthX = blocklengthY

def vicon2state(position):
    new_position = Point()
    try:
        assert(position.x >= base.x and position.x <= maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, base.x, maximum.x)
    except AssertionError:
        print "x position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (position.x, base.x, maximum.x, (base.x+ maximum.x)/2.)
        position.x = (base.x+ maximum.x)/2.

    try:
        assert(position.y >= base.y and position.y <= maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, base.y, maximum.y)
    except AssertionError:
        print "y position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (position.y, base.y, maximum.y, (base.y + maximum.y)/2.) 
        position.y = (base.y + maximum.y)/2.

    new_position.y = int((position.y - base.y)/blocklengthY)
    new_position.x = int((position.x - base.x)/blocklengthX)
    
    try:
        assert(new_position.x >= 0 and new_position.x <= X_NUMBER_TILE), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.x, 0, X_NUMBER_TILE)
        assert(new_position.y >= 0 and new_position.y <= Y_NUMBER_TILE), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.y, 0, Y_NUMBER_TILE)
    except AssertionError:
        new_position.x = 0
        new_position.y = 0

    state = floor(new_position.y) + (floor(new_position.x)*X_NUMBER_TILE)
    return int(state)

def state2vicon(self, state):
    xaxis = state / X_NUMBER_TILE
    yaxis = state % X_NUMBER_TILE

    position = Point()
    position.x = self.base.x + (self.blocklengthX/2.) + (xaxis * self.blocklengthX ) 
    position.y = self.base.y + (self.blocklengthY/2.) + (yaxis * self.blocklengthY ) 
    position.z = self.base.z
    return position
def send_trajectory(traj_3d, sampling_rate):
    r = rospy.Rate(sampling_rate)
    for i in range(len(traj_3d.time)):
        att_msg = AttPVA()
        att_msg.use_position = True
        att_msg.use_speed = True
        att_msg.use_acceleration = False
        att_msg.use_rate = False
        att_msg.use_yaw = True

        att_msg.yaw = 0
        att_msg.posZ_thrust =traj_3d.position[i].position.z
        att_msg.posY_pitch = traj_3d.position[i].position.y
        att_msg.posX_roll = traj_3d.position[i].position.x

        att_msg.velZ=traj_3d.velocity[i].linear.z
        att_msg.velY=traj_3d.velocity[i].linear.y
        att_msg.velX=traj_3d.velocity[i].linear.x

        att_msg.accZ=traj_3d.acceleration[i].force.z
        att_msg.accY=traj_3d.acceleration[i].force.y
        att_msg.accX=traj_3d.acceleration[i].force.x
        env_traj_pub.publish(att_msg)
        r.sleep()

def get_traj(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
    rospy.wait_for_service('path_planner')
    try:
        traj_srv = rospy.ServiceProxy('path_planner', Traj)
        resp1 = traj_srv(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq)
        return resp1.trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def handle_position(msg):
	global quad_current_pos
	quad_current_pos.x = msg.transform.translation.x
	quad_current_pos.y = msg.transform.translation.y
	quad_current_pos.z = msg.transform.translation.z

def handle_joy(msg):




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

if __name__ == "__main__":
	rospy.init_node('env_node', anonymous=True)
    rospy.Subscriber("/vicon"+ENV_NAME+ENV_NAME, TransformStamped, handle_position)
    rospy.Subscriber("/joy", Joy, handle_joy)

    env_traj_pub = rospy.Publisher('/qcontrol/att_pva', AttPVA, queue_size=10)


