#!/usr/bin/env python
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


###---- Visual Tracking and Servoing----
from transform import velocityTwistMatrix
from transform import homogenousMatrix
import visual_servoing as vs
import camera_parameters as cam

import time
import sys
import argparse

# ---------- Global Variables ---------------
global enable_depth
global enable_vs
global init_p0
global depth_p0
global depth_wrt_startup

# visual servoing
global desired_points_vs
global vcam_vs
global lambda_vs
global n_points_vs

vcam_vs = np.array([0,0,0,0,0,0])
lambda_vs = 0.5
n_points_vs = 8
desired_points_vs = []
enable_vs = 0   


    
set_mode = [0]*3
Vmax_mot = 1900
Vmin_mot = 1100

angle_wrt_startup = [0]*3
depth_wrt_startup = 0
depth_p0 = 0
rho = 1000.0
gravity = 9.80665

#Conditions
init_a0 = True
init_p0 = True
arming = False
custom_PI = False

set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

enable_depth = False 
enable_ping = False 
pinger_confidence = 0
pinger_distance = 0

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 



def trackercallback(data):
    global desired_points_vs
    global n_points_vs
    
    # read the current tracked point
    current_points = data.data
    
    # if we have current_points of same size as desired_points
    # then we can compute control law
    if(len(current_points)>0 and 
       len(desired_points_vs) == len(current_points)):
        
        #convert points from pixels to meters
        current_points_meter = cam.convertListPoint2meter (current_points)
        desired_points_meter = cam.convertListPoint2meter (desired_points_vs)
        
        #compute vs error
        error_vs = np.zeros((1,16))
        # error_vs = ......
       
        #compute interaction matrix in the FILE ./visual_servoig.py
        L = vs.interactionMatrixFeaturePoint2DList(current_points_meter)
        # TODO once it works with this matrix, change it for 
        # 1. a rho tetha representation
        # 2. a segment representation
        
        #init the camera velocity
        vcam = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        #TODO compute the velocity control law 
        # vcam_vs = ......
        
        ## Find the relative robot/camera position
        ## You can train in the file testTransform.py
        ## robot frame        |  camera frame 
        ##                    |
        ##    ------> x       |  -----> z
        ##    |               |  |
        ##    |               |  |
        ##    v z             |  v y
        ##                    |
    
        print( 'Visual servoing : vcam =', vcam_vs)
        vrobot = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
        ## TODO find the control velocity expressed in the robot frame
        ## vrobot = .........(vcam_vs)
    
        print('Then vrobot =  ', vrobot)
    
        vel = Twist()
        vel.angular.x = vrobot[3]
        vel.angular.y = vrobot[4]
        vel.angular.z = vrobot[5]
        vel.linear.x = vrobot[0]
        vel.linear.y = vrobot[1]
        vel.linear.z = vrobot[2]
        # publish the visual servoing velocity
        pub_visual_servoing_vel.publish(vel)
        
        # publish the error
        error_vs_msg = Float64MultiArray(data = error_vs)
        pub_visual_servoing_err.publish(error_vs_msg)
        
        print("press A to launch the visual servoing control")
        if (set_mode[2]):
           
            # Extract cmd_vel message
            # FIXME be carreful of the sign may depends on your robot 
            roll_left_right = mapValueScalSat(vel.angular.x)
            yaw_left_right = mapValueScalSat(-vel.angular.z)
            ascend_descend = mapValueScalSat(vel.linear.z)
            forward_reverse = mapValueScalSat(vel.linear.x)
            lateral_left_right = mapValueScalSat(-vel.linear.y)
            pitch_left_right = mapValueScalSat(vel.angular.y)

            setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                    yaw_left_right, forward_reverse, lateral_left_right)
    


    
def desiredpointscallback(data):
   # print( "desired point callback")
    global desired_points_vs
    desired_points_vs = data.data
    


def joyCallback(data):
    
    print ("joy callback") 
    global arming
    global set_mode
    global set_mode
    global custom_PI

    # Joystick buttons
    btn_arm = data.buttons[7]  # Start button
    btn_disarm = data.buttons[6]  # Back button
    btn_manual_mode = data.buttons[3]  # Y button
    btn_automatic_mode = data.buttons[2]   # X button
    btn_corrected_mode = data.buttons[0]    # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual, auto and correction mode
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
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        custom_PI = True        
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        print ("Armed wait for service mavros/cmd/command")
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



def pingerCallback(data):
    global pinger_confidence
    global pinger_distance

    if enable_ping == True : 
       pinger_distance = data.data[0]
       pinger_confidence = data.data[1]
       




def OdoCallback(data):
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r
   
    orientation = data.orientation
    angular_velocity = data.angular_velocity

    # extraction of yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angle_roll = euler[0]
    angle_pitch = euler[1]
    angle_yaw = euler[2]

    if (init_a0):
        # at 1st execution, init
        angle_roll_a0 = angle_roll
        angle_pitch_a0 = angle_pitch
        angle_yaw_a0 = angle_yaw
        init_a0 = False

    angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi

    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]

    pub_angle_degre.publish(angle)

    # Extraction of angular velocity
    p = angular_velocity.x
    q = angular_velocity.y
    r = angular_velocity.z

    vel = Twist()
    vel.angular.x = p
    vel.angular.y = q
    vel.angular.z = r
    pub_angular_velocity.publish(vel)



def DvlCallback(data):
    global set_mode
    global custom_PI
    global u
    global v
    global w
  
    u = data.velocity.x  # Linear surge velocity 
    v = data.velocity.y  # Linear sway velocity
    w = data.velocity.z  # Linear heave velocity

    Vel = Twist()
    Vel.linear.x = u
    Vel.linear.y = v
    Vel.linear.z = w
    pub_linear_velocity.publish(Vel)
    
#-------------------------Control------------------------------
    #if(custom_PI):.........................
      
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        #setOverrideRCIN ( Pitch , Roll , Heave , Yaw ,Surge, Sway)
        setOverrideRCIN(1500, 1500, 1700, 1500, 1500, 1500)
        return






def PressureCallback(data):
    global depth_p0
    global depth_wrt_startup
    global init_p0
    global enable_depth
  
 
    if(enable_depth):
        pressure = data.fluid_pressure

        if (init_p0):
            # 1st execution, init
            depth_p0 = (pressure - 101300)/(rho* gravity)
            init_p0 = False 
        depth_wrt_startup = (pressure - 101300)/(rho * gravity) - depth_p0

        # publish depth_wrt_startup data
        msg = Float64()
        msg.data = depth_wrt_startup
        pub_depth.publish(msg)

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        #setOverrideRCIN ( Pitch , Roll , Heave , Yaw ,Surge, Sway)
        setOverrideRCIN(1500, 1500, 1500, 1500, 1700, 1500)
        return


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

    msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
    msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll 	
    msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
    msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw		    
  
    msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge		
    
    msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500
    
    # print("<3=====D ",msg_override)
    pub_msg_override.publish(msg_override)



# Function used to calculate the necessary PWM for each motor
def PWM_Cmd(thrust_req):
    if (thrust_req >= 0):
        m = 86.93393326839376    # Slope of the positive force
        b = 1536
    else:
        m = 110.918185437553874  # Slope of the negtaive force 
        b = 1464

    PWM = int(m * thrust_req/4) + b
    if PWM > Vmax_mot:
        PWM = Vmax_mot
    if PWM < Vmin_mot:
        PWM = Vmin_mot

    return PWM        
    

def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
    rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
    #camera
    rospy.Subscriber("tracked_points",Float64MultiArray,trackercallback, queue_size=1)
    rospy.Subscriber("desired_points",Float64MultiArray,desiredpointscallback, queue_size=1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('visual_servoing_mir', anonymous=False)
    print "visual servoing mir launched"
    #armDisarm(False)  # Not automatically disarmed at startup
    pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
    pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
    pub_depth = rospy.Publisher('depth/state`', Float64, queue_size = 10, tcp_nodelay = True)
    
    pub_visual_servoing_vel = rospy.Publisher('visual_servoing_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    pub_visual_servoing_err = rospy.Publisher("visual_servoing_error",Float64MultiArray,queue_size=1,tcp_nodelay = True)
    
    pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    
    pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    
    subscriber()


