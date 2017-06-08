import rospy

from qcontrol_defs.msg import AttPVA
from qcontrol_defs.msg import QuadState
from openbci.msg import Action
from qcontrol_defs.srv import *
from geometry_msgs.msg import Point, TransformStamped
from time import sleep
import sys


current_state = QuadState()

def takeoff():
    rospy.wait_for_service('/Quad10/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.start_takeoff = CommandActionRequest.START_TAKEOFF_TRUE
    cmd_srv = rospy.ServiceProxy('/Quad10/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

    wait_rate = rospy.Rate(50)
    while not current_state.takeoff_complete :
        wait_rate.sleep()

def start_position_control():
    rospy.wait_for_service('/Quad10/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.arm_motors = CommandActionRequest.ARM_MOTOR_TRUE
    cmdAction.is_posctl = CommandActionRequest.IS_POSCTL_TRUE
    cmd_srv = rospy.ServiceProxy('/Quad10/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

def land():
    rospy.wait_for_service('/Quad10/qcontrol/commands')
    cmdAction = CommandActionRequest()
    cmdAction.start_landing = CommandActionRequest.START_LANDING_TRUE;
    cmd_srv = rospy.ServiceProxy('/Quad10/qcontrol/commands', CommandAction)
    reponse = cmd_srv(cmdAction)

def send_position(pos):
    att_msg = AttPVA()
    att_msg.use_position = True
    att_msg.posZ_thrust = pos.z
    att_msg.posY_pitch = pos.y
    att_msg.posX_roll = pos.x
    att_msg.use_speed = False

    att_msg.use_yaw = True
    att_msg.yaw = -0.07

    #att_msg.use_rate = False
    #att_msg.velZ = val
    #att_msg.velX = val
    #att_msg.velY = val
    #and so on

    pub.publish(att_msg)

def quadStateSub(state):
    global current_state
    current_state = state

def action_sub(msg):
    global current_pos
    global pos
    print msg
    if msg.rand == 'south':
        pos.x = -0.9
        pos.y = current_pos.y
        pos.z = current_pos.z - .33
    elif msg.rand == 'north':
        pos.x = -0.9
        pos.y = current_pos.y
        pos.z = current_pos.z + .33
    elif msg.rand == 'stay':
        pos.x = -0.9
        pos.y = current_pos.y
        pos.z = current_pos.z
    elif msg.rand == 'west':
        pos.x = -0.9
        pos.y = current_pos.y + 0.75
        pos.z = current_pos.z
    elif msg.rand == 'east':
        pos.x = -0.9
        pos.y = current_pos.y - 0.75
        pos.z = current_pos.z

    elif msg.rand == 'land':
        land()

    if pos.x > 1:
        pos.x = 1
    if pos.x < -1:
        pos.x = -1
    if pos.y > 1:
        pos.y = 1
    if pos.y < -1:
        pos.y = -1
    if pos.z > 1.8:
        pos.z = 1.8
    if pos.z < 0.8:
        pos.z = .8
def update_location(msg):
    global vicon_pos

    vicon_pos.x = msg.transform.translation.x
    vicon_pos.y = msg.transform.translation.y
    vicon_pos.z = msg.transform.translation.z

if __name__ == "__main__":
    rospy.init_node('commander', anonymous=True)

    vicon_pos = Point()

    current_pos = Point()


    pos = Point()

    rospy.Subscriber('/openbci/live_cmd',Action,action_sub)

    rospy.Subscriber('/vicon/Quad10/Quad10',TransformStamped,update_location)
    

    #Topic where you can listen to the current state of the quad. Attribute of QuadState are :
    ### is_offboard = true if quad is in offboard mode and false if it is in AUTO.LOITER mode (security)
    ### is_attctl = true if quad is in attitude control mode
    ### is_posctl = true if quad is in position control mode
    ### is_armed = true if the motors are armed
    ### is_landed = true if the quad is currently on the ground
    ### is_takingoff = true if the quad is trying to take off
    ### takeoff_complete = true if the quad has reach the takeoff altitude
    rospy.Subscriber('/Quad10/qcontrol/quad_info',QuadState,quadStateSub)

    #Topic where you can send PVA to quad. Attribute of AttPVA are :
    ### use_position = true if want to use posX,posY,posZ attributes of the message
    ### use_speed = true if want to use velX,velY and velZ attributes of the message
    ### use_acceleartion = true if want to use accX,accY and accZ attributes of the message
    ### use_yaw = true if want to use yaw value attribute of the message
    ### use_body_frame_offset = true if want to use the quad current heading as reference (mainly useful for posctrl)
    ### use_rate = true for using rate value instead of angle in attitude control and position control mode
    ### posZ_thrust : posZ or thrust depending of attctl or posctl mode
    ### posX_roll : posX or roll depending of attctl or posctl mode
    ### posY_pitch : posY or pitch depending of attctl or posctl mode
    ### velX 
    ### velY
    ### velZ
    ### accX
    ### accY
    ### accZ
    pub = rospy.Publisher('/Quad10/qcontrol/att_pva', AttPVA, queue_size=10)

    #Starting position control
    #start_position_control()

    ###Even better to takeoff before starting position control by this:
    #takeoff()
    start_position_control()

    wait_rate = rospy.Rate(10)

    vicon_pos = Point()

    current_pos = Point()
    current_pos.x = -0.9
    current_pos.y = -0.6
    current_pos.z = 1.4

    pos.x =-0.9
    pos.y = -0.6
    pos.z = 1.4

    while not rospy.is_shutdown():
        send_position(pos)
        if (vicon_pos.x - pos.x < .1) and (vicon_pos.y - pos.y < .1) and (vicon_pos.z - pos.z < .1):
            current_pos = pos
        wait_rate.sleep()