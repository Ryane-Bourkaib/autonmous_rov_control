#!/usr/bin/env python
import math
from os import kill
import string

import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
from autonomous_rov.msg import pwm
from alpha_beta_gamma_filter import alpha_beta_gamma_filter
from PI_Controller import*
# from brping import Ping1D
import time
import sys
import argparse


# ---------- Global Variables ---------------
global enable_depth
global init_p0
global counter

global depth_p0
global depth_wrt_startup
global custom_SM
global custom_PI

global wall_dist_pwm
global depth_pwm
global yaw_pwm
global sway_pwm


set_mode = [0]*3
Vmax_mot = 1900
Vmin_mot = 1100

angle_wrt_startup = [0]*3
depth_wrt_startup = 0
depth_p0 = 0
counter = 0

# Conditions
init_a0 = True
init_p0 = True
enable_depth = True  # Don't Publish the depth data until asked
arming = False
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

custom_PI = False
do_surge = True
enable_ping = True
pinger_confidence = 0
pinger_distance = 0

# Linear and angular velocities
u = 0  # Initial linear surge velocity
v = 0  # Initial linear sway velocity
w = 0  # Initial linear heave velocity

p = 0  # angular  velocity along x
q = 0  # angular velocity along y
r = 0  # angular velocity along z


def joyCallback(data):
    global depth_p0
    global arming
    global set_mode
    global init_a0
    global init_p0
    global mode
    global custom_PI

    # Joystick buttons
    btn_arm = data.buttons[7]           # Start button
    btn_disarm = data.buttons[6]        # Back button
    btn_manual_mode = data.buttons[3]    # Y button
    btn_automatic_mode = data.buttons[2]  # X button
    btn_corrected_mode = data.buttons[0]  # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual and auto mode
    if (btn_manual_mode and not set_mode[0]):
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False
        custom_SM = False
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False
        custom_SM = False
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        init_a0 = True
        init_p0 = True
        # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        custom_PI = True
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except disarming")


def velCallback(cmd_vel):

    global set_mode
    # Only continue if manual_mode is enabled
    if (set_mode[1] or set_mode[2]):
        return

    # Extract cmd_vel message
    roll_left_right = mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(-cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                    yaw_left_right, forward_reverse, lateral_left_right)


rho = 1000.0
gravity = 9.80665


# PID Parameters :
I0_x, I0_y, I0_z, I0_theta, I0_psi = 0, 0, 0, 0, 0  # Initial value of integral
e0_x, e0_y, e0_z, e0_psi = 0, 0, 0, 0  # Initial error

# Filter gains :
alpha = 0.45
beta = 0.1
u_e0, ud_e0 = 0, 0  # Estimated surge velocity/position
vd_e0, v_e0 = 0, 0  # Estimated sway velocity/position

u_d = 0  # Desired surge velocity
yaw_d = 0  # Desired yaw angle


def mapValueScalSat(value):
    global Vmax_mot
    global Vmin_mot
    # Correction_Vel and joy between -1 et 1
    # scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # neutral point is 1500
    pulse_width = value * 400 + 1500

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot

    return pulse_width


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    # This function replaces setservo for motor commands.
    # It overrides Rc channels inputs and simulates motor controls.
    # In this case, each channel manages a group of motors not individually as servo set

    msg_override = OverrideRCIn()

    msg_override.channels[0] = np.uint(
        channel_pitch)       # pulseCmd[4]--> pitch
    msg_override.channels[1] = np.uint(
        channel_roll)        # pulseCmd[3]--> roll
    msg_override.channels[2] = np.uint(
        channel_throttle)    # pulseCmd[2]--> heave
    msg_override.channels[3] = np.uint(channel_yaw)        # pulseCmd[5]--> yaw

    msg_override.channels[4] = np.uint(
        channel_forward)     # pulseCmd[0]--> surge

    msg_override.channels[5] = np.uint(
        channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500

    # print("<3=====D ",msg_override)
    pub_msg_override.publish(msg_override)


def DoThing(msg):
    print(msg.data)
    setOverrideRCIN(1500, 1500, 1500, 1500, msg.data, 1500)

####################Functions######################################

# Function used to calculate the necessary PWM for each motor

def PWM_Cmd(thrust_req):
    if (thrust_req >= 0):
        m = 86.93393326839376   # Slope of the positive force linear function
        b = 1536
    else:
        m = 110.918185437553874  # Slope of the negtaive force linear function
        b = 1464

    PWM = int(m * thrust_req/4) + b
    if PWM > Vmax_mot:
        PWM = Vmax_mot
    if PWM < Vmin_mot:
        PWM = Vmin_mot
    return PWM

# Function used to enble the depth calback


def EnableDepthCallback(msg):
    global enable_depth
    global init_p0
    global counter
    counter = 0
    enable_depth = True
    init_p0 = True


def wall_dist_cb(msg):
    global wall_dist_pwm
    wall_dist_pwm = PWM_Cmd(msg.data)


def yaw_cb(msg):
    global yaw_pwm
    yaw_pwm = PWM_Cmd(msg.data)


def depth_cb(msg):
    global depth_pwm
    depth_pwm = PWM_Cmd(msg.data)


def sway_cb(msg):
    global sway_pwm
    sway_pwm = PWM_Cmd(msg.data)

def main_cb():
    # all logic should go here
    pass


def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    
    rospy.Subscriber("enable_depth", Empty, EnableDepthCallback)
    rospy.Subscriber("do/thing", Int16, DoThing)

    # Controllers Subscribers
    rospy.Subscriber("controller/surge/effort", Float64, wall_dist_cb)
    rospy.Subscriber("controller/yaw/effort", Float64, yaw_cb)
    rospy.Subscriber("controller/depth/effort", Float64, depth_cb)
    rospy.Subscriber("controller/sway/effort", Float64, sway_cb)
    
    hz = 50.0
    rospy.Timer(rospy.Duration.from_sec(1/hz), main_cb)

    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
    pub_msg_override = rospy.Publisher(
        "mavros/rc/override", OverrideRCIn, queue_size=10, tcp_nodelay=True)
    pub_angle_degre = rospy.Publisher(
        'angle_degree', Twist, queue_size=10, tcp_nodelay=True)
    pub_depth = rospy.Publisher(
        'depth/state', Float64, queue_size=10, tcp_nodelay=True)
    pub_angular_velocity = rospy.Publisher(
        'angular_velocity', Twist, queue_size=10, tcp_nodelay=True)
    pub_linear_vel = rospy.Publisher(
        'linear_velocity', Twist, queue_size=10, tcp_nodelay=True)

    subscriber()
