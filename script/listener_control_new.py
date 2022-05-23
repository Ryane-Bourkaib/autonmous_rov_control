#!/usr/bin/env python
from doctest import master
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

set_mode = [0]*3
Vmax_mot = 1900
Vmin_mot = 1100

# Conditions
init_a0 = True
init_p0 = True
enable_depth = True  # Don't Publish the depth data until asked
arming = False
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction


def joyCallback(data):
    global arming
    global set_mode

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
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
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


class Master:
    def __init__(self):
        
        rospy.Subscriber("controller/surge/effort", Float64, self.surge_cb)
        rospy.Subscriber("controller/yaw/effort", Float64, self.yaw_cb)
        rospy.Subscriber("controller/depth/effort", Float64, self.depth_cb)
        rospy.Subscriber("controller/sway/effort", Float64, self.sway_cb)
        
        self.depth_setpoint_pub = rospy.Publisher("controllers/depth/desired",Float64,queue_size=10)
        self.yaw_setpoint_pub = rospy.Publisher("controllers/yaw/desired",Float64,queue_size=10)
        self.surge_setpoint_pub = rospy.Publisher("controllers/surge/desired",Float64,queue_size=10)
        self.sway_setpoint_pub = rospy.Publisher(
            "controllers/sway/desired", Float64, queue_size=10)

        self.surge_pwm = 1500
        self.depth_pwm = 1500
        self.sway_pwm = 1500
        self.yaw_pwm = 1500
        self.free_dist_thresh = 700
        self.depth_desired = 0.5
        self.yaw_desired = 0.0
        self.surge_desired = 0.5
        self.sway_desired = 0.0
        
        self.surge = 0
        self.yaw = 0
        self.sway = 0
        self.depth = 0
        self.state_names = ["drown", "go_to_wall", "keep_to_wall", "search"]
        # self.state_nums = {name:val for val, name in self.state_names.items()}
        self.actions = {"drown": self.drown_action,
                        "go_to_wall": self.go_to_wall_action,
                        "keep_to_wall": self.keep_to_wall_action,
                        "search": self.search_action}
        self.state = 0
        self.prev_state = None
        
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                            surge=self.surge_desired, sway=self.sway_desired)

    def surge_cb(self, msg):
        self.surge_pwm = self.PWM_Cmd(msg.data[0])
        self.surge = msg.data[1]
        self.update_state()

    def yaw_cb(self, msg):
        self.surge_pwm = self.PWM_Cmd(msg.data[0])
        self.yaw = msg.data[1]
        self.update_state()

    def depth_cb(self, msg):
        self.depth_pwm = self.PWM_Cmd(msg.data[0])
        self.depth = msg.data[1]
        self.update_state()

    def sway_cb(self, msg):
        self.sway_pwm = self.PWM_Cmd(msg.data[0])
        self.sway = msg.data[1]
        self.update_state()

    # Function used to calculate the necessary PWM for each motor

    def PWM_Cmd(self, thrust_req):
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

    def send_setpoints(self, depth, yaw, surge, sway):
        self.depth_setpoint_pub.publish(Float64(depth))
        self.yaw_setpoint_pub.publish(Float64(yaw))
        self.surge_setpoint_pub.publish(Float64(surge))
        self.sway_setpoint_pub.publish(Float64(sway))
    
    def update_state(self):
        self.actions[self.state]
        
        if abs(abs(self.depth) - self.depth_desired) > 0.1:
            self.state = "drown"
            return
        
        elif self.state == "drown":
            if abs(abs(self.depth) - self.depth_desired) <= 0.1:
                self.state = "go_to_wall"
            return
        
        elif self.state == "go_to_wall":
            if abs(self.surge - self.surge_desired) < 100:
                self.state = "keep_to_wall"
            return
        
        elif self.state == "keep_to_wall":
            self.state = "search"
            #self.state = "bonus"
            return
        
        elif self.state == "search":
            if self.surge >= self.free_dist_thresh:
                self.state = "go_to_wall"
            return           

    def drown_action(self):
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, 1500, self.sway_pwm)
    
    def go_to_wall_action(self):
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, self.surge, self.sway_pwm)
        
    def keep_to_wall_action(self):
        setOverrideRCIN(1500, 1500, self.depth_pwm,
                        self.yaw_pwm, self.surge, self.sway_pwm)
    
    def search_action(self):
        self.yaw_desired -= 0.5
        self.send_setpoints(depth=self.depth_desired, yaw=self.yaw_desired,
                    surge=self.surge_desired, sway=self.sway_desired)
        setOverrideRCIN(1500, 1500, self.depth_pwm,self.yaw_pwm, 1500, 1500)


def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("do/thing", Int16, DoThing)

    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('autonomous_MIR', anonymous=False)
    
    master = Master()
    
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
